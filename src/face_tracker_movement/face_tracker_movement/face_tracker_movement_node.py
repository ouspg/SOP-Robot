import math
import sys
import time
import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from face_tracker_msgs.msg import Point2, Faces
from std_msgs.msg import String, Float32


class FaceTrackerMovementNode(Node):

    def __init__(self):
        super().__init__('face_tracker_movement_client')

        functionality = (
            self.declare_parameter("functionality", "full")
            .get_parameter_value()
            .string_value
        )

        simulation = (
            self.declare_parameter("simulation", False)
            .get_parameter_value()
            ._bool_value
        )
        
        self.logger = self.get_logger()

        # Action clients for eye and head controllers
        self.eye_action_client = ActionClient(self, FollowJointTrajectory, '/eyes_controller/follow_joint_trajectory')
        self.head_action_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

        # ROS2 subscriptions
        self.face_list_subscription = self.create_subscription(Faces, '/face_tracker/faces', self.face_list_callback, 2)
        self.head_state_subscription = self.create_subscription(JointTrajectoryControllerState, '/head_controller/controller_state', self.head_state_callback, 5)
        self.eyes_state_subscription = self.create_subscription(JointTrajectoryControllerState, '/eyes_controller/controller_state', self.eyes_state_callback, 5)

        self.head_gesture_subscription = self.create_subscription(String, '/face_tracker_movement/head_gesture_topic', self.head_gesture_callback, 10)

        # Set initial values
        if simulation:
            self.logger.info("Running face_tracker_movement in simulation mode")

            # Set initial values for simulation environment
            # Movement limits
            self.head_vertical_lower_limit = 0
            self.head_vertical_upper_limit = 2
            self.head_pan_lower_limit = -2.0
            self.head_pan_upper_limit = 2.0
            self.eye_vertical_lower_limit = -0.5
            self.eye_vertical_upper_limit = 0.5
            self.eye_horizontal_lower_limit = -0.6
            self.eye_horizontal_upper_limit = 0.6

            # starting states
            self.start_head_state = [0, 0, 0, 1]                # Should not be modified in runtime.
            self.eyes_center_position = [0, 0]                  # Should not be modified in runtime.

            # Set movement directions
            self.head_pan_multiplicator = -1
            self.head_vertical_multiplicator = 1
            self.eye_vertical_multiplicator = -1
            self.eye_horizontal_multiplicator = -1

        else:
            self.logger.info(f"Running face_tracker_movement in hardware mode")

            # Set initial values for actual robot
            # Movement limits
            self.head_vertical_lower_limit = 0.8
            self.head_vertical_upper_limit = 1.5
            self.head_pan_lower_limit = -0.25
            self.head_pan_upper_limit = 1.75
            self.eye_vertical_lower_limit = -0.7
            self.eye_vertical_upper_limit = -0.2
            self.eye_horizontal_lower_limit = -2.0
            self.eye_horizontal_upper_limit = 0.0

            # starting states
            self.start_head_state = [0.6, 0.5, -0.5, 1.2]       # Should not be modified in runtime.
            self.eyes_center_position = [-0.7, -0.75]           # Should not be modified in runtime.

            # Set movement directions
            self.head_pan_multiplicator = 1
            self.head_vertical_multiplicator = 1
            self.eye_vertical_multiplicator = 1
            self.eye_horizontal_multiplicator = 1


        # Middle point of image view
        self.middle_x = 640
        self.middle_y = 400
        self.is_glancing = False
        self.idling = False # Not used currently

        self.head_joint_ids = [4, 1, 3, 2]              # Servo ids for head joints. Order comes from head_controller: [head_pan_joint, head_tilt_right_joint, head_tilt_left_joint, head_tilt_vertical_joint]
        self.head_state = self.start_head_state[:]      # Tries to have the up-to-date head servo values.

        self.eyes_joint_ids = [9, 11]                   # Servo ids for eye joints. Order comes from eyes_controller: [eyes_shift_horizontal_joint, eyes_shift_vertical_joint]
        self.eyes_state = self.eyes_center_position[:]      # Tries to have the up-to-date head servo values.

        # Some variables
        self.pan_diff = 0
        self.goal_pan = self.head_state[0]
        self.v_diff = 0
        self.goal_vertical_tilt = self.head_state[3]

        # Are head/eye joints used in the current configuration? 
        self.head_enabled = True
        self.eyes_enabled = True

        # Face info
        self.visible_face_amount = 0
        self.previous_face = None  # TODO: Not yet used
        self.preferred_face_id = None
        self.preferred_face_tracking_start_time = None

        # TODO: Consider using non constant value for preferred face tracking time limit
        self.preferred_face_tracking_timelimit = 10  # in seconds

        self.blocked_faces: list[dict] = []  # Includes face_id, block_start and block_duration

        # TODO: Consider using non constant value for face blocking time
        self.blocked_faces_timeout = 20  # in seconds

        # Face tracking tresholds
        # TODO: Select good values
        # TODO: use ros arguments for the treshold values to be able to change these on runtime
        self.ignore_small_faces_treshold = 50
        self.near_interaction_treshold = 320

        if functionality.lower() == "head":
            self.logger.info('Eye movement is disabled.')
            self.eyes_enabled = False
        elif functionality.lower() == "eyes":
            self.logger.info('Head movement is disabled.')
            self.head_enabled = False

        self.center_eyes()
        self.send_head_goal(self.head_state[0], self.head_state[3], self.head_state[1])
        time.sleep(1)

        self.idle_timer = self.create_timer(5, self.idle_timer_callback)

        self.logger.info('Face tracking movement client initialized.')

    # Main loop. Excecuted when face_tracker_node publishes faces.
    def face_list_callback(self, msg):
        self.visible_face_amount = len(msg.faces)

        # TODO: Make more sophisticated function for face movement

        # TODO Propably not needed to update every time (as goal is 30fps). Consider moving this to own timer or something.
        self.update_blocked_faces()

        face = self.select_face_to_track(msg.faces)

        if face:
            # Calculate middle coordinates
            x=round((face.top_left.x + face.bottom_right.x) / 2)
            y=round((face.top_left.y + face.bottom_right.y) / 2)

            # Reset idle callback
            self.idle_timer.timer_period_ns = 2000000000  # 2s
            self.idle_timer.reset()
            self.idling = False

            self.analyze_coordinates(x, y)
        
    # Get the current state of head joints. Updated at 20 Hz (see robot.yaml)
    def head_state_callback(self, msg):
        for i, val in enumerate(msg.actual.positions):
            if math.isnan(val):
                self.head_state[i] = self.start_head_state[i]
                self.logger.info("Head joint ID " + str(self.head_joint_ids[i]) + " is not responding")
            else:
                self.head_state[i] = val

    # Get the current state of eye joints. Updated at 20 Hz (see robot.yaml)
    def eyes_state_callback(self, msg):
        for i, val in enumerate(msg.actual.positions):
            if math.isnan(val):
                self.eyes_state[i] = self.eyes_center_position[i]
                self.logger.info("Eye joint ID " + str(self.eyes_joint_ids[i]) + " is not responding")
            else:
                self.eyes_state[i] = val

    def head_gesture_callback(self, msg):
        gesture = msg.data
        self.logger.info(f"Head gesture: {gesture}")
        gesture = gesture.split(",")
        args = {}
        for i in range(1, len(gesture)):
            gesture[i] = gesture[i].strip(" ")
            if gesture[i][:10] == "magnitude=":
                args['magnitude'] = float(gesture[i][10:])
            if gesture[i][:6] == "delay=":
                args['delay'] = float(gesture[i][6:])
            if gesture[i][:9] == "duration=":
                args['duration_of_individual_movements'] = float(gesture[i][9:])
        gesture = gesture[0]
        if gesture == 'nod':
            self.nod(**args)
        elif gesture == 'shake':
            self.head_shake(**args)
        else:
            self.logger.info("Gesture not implemented!")

    # Get random horizontal positions for eyes and head and turn there at a random speed
    def idle_timer_callback(self):
        self.idling = True
        self.logger.info("Idling...\x1B[1A")
        if self.eyes_enabled:
            self.send_eye_goal(self.get_random_eye_location()[0], self.eyes_center_position[1])

        if self.head_enabled:
            self.goal_pan = random.uniform(self.head_pan_lower_limit, self.head_pan_upper_limit)
            self.send_pan_and_vertical_tilt_goal(self.goal_pan, self.start_head_state[3], Duration(sec=0, nanosec= random.randint(1000000000, 4000000000)))
        self.idle_timer.timer_period_ns = random.randint(1000000000, 4000000000)
        self.idle_timer.reset()

    def select_face_to_track(self, faces):
        """
        Selects and tracks a face from a list based on preferred face ID or largest face.
        
        Args:
            faces (list): List of face objects.
        
        Returns:
            face: The selected face object, or `None` if no suitable face is found.
        """
        non_blocked_faces = []
        selected_face = None
    
        # TODO Include face.speaking check to the algorithm
        
        # Check face sizes to determine if the person should be tracked constantly
        # TODO: Consider to use information also from other modules - like if somebody speaks to the robot
        max_face_size = max([face.diagonal for face in faces])
        if max_face_size > self.near_interaction_treshold:
            # Near interaction - Continuous tracking

            # remove preferred face

            selected_face = self.select_largest_face(faces)
        else:
            # Far interaction - Look faces only some time and then ignore

            # Remove blocked faces and too small faces from faces list
            for face in faces:
                if ((face.diagonal > self.ignore_small_faces_treshold) and
                    (face.face_id not in [blocked_face["face_id"] for blocked_face in self.blocked_faces])):
                    non_blocked_faces.append(face)

            # face list can now be empty
            if len(non_blocked_faces) == 0:
                return None
            
            selected_face = self.select_face_far_interaction(non_blocked_faces)

        # selected_face can be None
        if selected_face:
            if not self.previous_face or selected_face.face_id != self.previous_face.face_id:
                self.logger.info(f"Tracking face with id={selected_face.face_id}")
            if not self.preferred_face_id:
                self.add_preferred_face(selected_face)
        self.previous_face = selected_face
        return selected_face
    
    def select_face_far_interaction(self, faces):
        if self.preferred_face_id:
            selected_face = next((face for face in faces if face.face_id == self.preferred_face_id), None)
            # Track preferred face if 
            if selected_face:
                if time.time() - self.preferred_face_tracking_start_time < self.preferred_face_tracking_timelimit:
                    # Select preferred face
                    pass
                else:
                    if len(faces) > 1:
                        # Select largest face that is not preferred face
                        face_list = [face for face in faces if face.face_id != self.preferred_face_id]
                        selected_face = self.select_largest_face(face_list)
                    else:
                        # No faces that can be selected
                        selected_face = None
                    self.remove_preferred_face()
            else:
                self.remove_preferred_face()

                # Select largest face
                selected_face = self.select_largest_face(faces)
        else:
            # Select largest face
            selected_face = self.select_largest_face(faces)

        return selected_face

    def select_largest_face(self, faces):
        return max(faces, key=lambda face: face.diagonal)
    
    def update_blocked_faces(self):
        remaining_faces = []

        # Filter and log deleted items
        for blocked_face in self.blocked_faces:
            if blocked_face["block_start"] + blocked_face["block_duration"] < time.time():
                self.logger.info(f"Removed face {blocked_face['face_id']} from blocked faces")
            else:
                remaining_faces.append(blocked_face)
        self.blocked_faces = remaining_faces

    def add_preferred_face(self, face):
        if self.preferred_face_id == face.face_id:
            self.logger.warning("Setting preferred face to same as old preferred face!")
        self.preferred_face_id = face.face_id
        self.preferred_face_tracking_start_time = face.occurances[-1].end_time  # Timestamp for last time face was seen
        self.logger.info(f"Preferred face set to {self.preferred_face_id}")

    def remove_preferred_face(self):
        self.blocked_faces.append({
            "face_id": self.preferred_face_id,
            "block_start": time.time(),
            "block_duration": self.blocked_faces_timeout,
        })
        self.logger.info(f"preferred face removed and blocked: {self.preferred_face_id}")
        self.preferred_face_id = None
        self.preferred_face_tracking_start_time = None

    # Feel free to experiment with the timings to fine-tune behavior
    def analyze_coordinates(self, x, y):

        if self.eyes_enabled:
            #self.logger.info('x: %d, y: %d' % (msg.x, msg.y))
            glance_percentage = 0.005 # Chance of executing a glance on each frame where a face has been detected. TODO: Decide on a good value
            randomvalue = random.uniform(0, 1)  

            # Check if doing the glance or not
            # TODO: Consider using head gestures for the glance
            if randomvalue <= glance_percentage:
                eye_location_x, eye_location_y = self.get_random_eye_location(distance_from_current_x_position = 0.5)
                self.is_glancing = True
                self.logger.info('glance')
            else:
                eye_location_x, eye_location_y = self.transform_face_location_to_eye_location(x, y)
                self.is_glancing = False

            # Move eyes
            self.send_eye_goal(eye_location_x, eye_location_y)

            if self.visible_face_amount > 1:
                time.sleep(0.5)

            if self.is_glancing:
                # Center the eyes back to the face after glancing
                time.sleep(0.5)
                self.center_eyes()
                time.sleep(0.7)
                return
        
        if self.head_enabled:
            self.goal_pan, self.goal_vertical_tilt = self.transform_face_location_to_head_values(x, y)
            self.pan_diff = self.goal_pan - self.head_state[0]
            self.v_diff = self.goal_vertical_tilt - self.head_state[3]
            if self.pan_diff != 0 or self.v_diff != 0:
                self.send_pan_and_vertical_tilt_goal(self.goal_pan, self.goal_vertical_tilt)
                time.sleep(0.3)
        
        time.sleep(0.2)

    def nod(self, magnitude=0.4, delay=0.5, duration_of_individual_movements=0.4):
        """
        Makes the head nod.
        Takes two optional arguments:
            magnitude: The interval between the highest and lowest points of the nod
            delay: The delay (in seconds) between the start of each individual movement
            duration_of_individual_movements: The time (in seconds) it takes for an individual movement to finish. Should be less than delay.
        """
        if self.head_state:
            verticalTilt_start = self.head_state[3]
            msg = Float32()
            msg.data = delay * 3
            # self.head_gesture_length_publisher.publish(msg)

            self.fixed_gaze_head_turn('up', magnitude / 2, duration_of_individual_movements)
            time.sleep(delay)
            self.fixed_gaze_head_turn('down', magnitude, duration_of_individual_movements)
            time.sleep(delay)
            verticalTilt = self.head_state[3]
            self.fixed_gaze_head_turn('up', verticalTilt - verticalTilt_start, duration_of_individual_movements)

    def head_shake(self, magnitude=0.5, delay=0.5, duration_of_individual_movements=0.4):
        """
        Shakes the head.
        Takes two optional arguments:
            magnitude: The interval between the leftmost and rightmost points of the head shakes
            delay: The delay (in seconds) between the start of each individual movement
            duration_of_individual_movements: The time (in seconds) it takes for an individual movement to finish. Should be less than delay.
        """
        if self.head_state:
            pan_start = self.head_state[0]
            msg = Float32()
            msg.data = delay * 3
            # self.head_gesture_length_publisher.publish(msg)
            self.fixed_gaze_head_turn('left', magnitude / 2, duration_of_individual_movements)
            time.sleep(delay)
            self.fixed_gaze_head_turn('right', magnitude, duration_of_individual_movements)
            time.sleep(delay)
            pan = self.head_state[0]
            self.fixed_gaze_head_turn('left', pan_start - pan, duration_of_individual_movements)

    def fixed_gaze_head_turn(self, direction, magnitude, duration=0.4):
        """
        Turns the head while keeping the gaze steady by rotating the eyes in the opposite direction as the head.
        Decreases the magnitude of the movement, if it would exceed the maximum range of either the head or the eyes. 
        Takes 3 arguments: direction, magnitude and duration.
            direction: The direction of the head turn. Can be "left", "right", "up" or "down".
            magnitude: The magnitude of the head turn.
            duration: The time in seconds it should take to finish the head turn.
        """
        duration = Duration(sec=0, nanosec=int(duration * 100000000))
        pan, _, _, verticalTilt = self.head_state
        eye_x, eye_y = self.eyes_state

        if direction == 'left':
            if pan + magnitude > self.head_pan_upper_limit:
                magnitude = self.head_pan_upper_limit - pan
            if eye_x - magnitude < self.eye_horizontal_lower_limit:
                magnitude = self.eye_horizontal_lower_limit - eye_x

            self.send_pan_and_vertical_tilt_goal(pan + magnitude, verticalTilt, duration)
            self.send_eye_goal(eye_x - magnitude, eye_y, duration)
        
        elif direction == 'right':
            if pan - magnitude < self.head_pan_lower_limit:
                magnitude = pan - self.head_pan_lower_limit
            if eye_x + magnitude > self.eye_horizontal_upper_limit:
                magnitude = self.eye_horizontal_upper_limit - eye_x
            self.send_pan_and_vertical_tilt_goal(pan - magnitude, verticalTilt, duration)
            self.send_eye_goal(eye_x + magnitude, eye_y, duration)
        
        elif direction == 'up':
            if verticalTilt - magnitude < self.head_vertical_lower_limit:
                magnitude = verticalTilt - self.head_vertical_lower_limit
            if eye_y + magnitude > self.eye_vertical_upper_limit:
                magnitude = self.eye_vertical_upper_limit - eye_y

            self.send_pan_and_vertical_tilt_goal(pan, verticalTilt - magnitude, duration)
            self.send_eye_goal(eye_x, eye_y + magnitude)
        
        elif direction == 'down':
            if verticalTilt + magnitude > self.head_vertical_upper_limit:
                magnitude = self.head_vertical_upper_limit - verticalTilt
            if eye_y - magnitude < self.eye_vertical_lower_limit:
                magnitude = eye_y - self.eye_vertical_lower_limit

            self.send_pan_and_vertical_tilt_goal(pan, verticalTilt + magnitude, duration)
            self.send_eye_goal(eye_x, eye_y - magnitude)

    def send_eye_goal(self, horizontal, vertical, duration=None):
        # The eyes lock up if they try to move too fast so it'll go a bit slower for longer movements (also faster for short movements)
        if duration == None:
            x_diff = abs(self.eyes_state[0] - horizontal)
            duration = Duration(sec=0, nanosec=max(int(200000000 * x_diff), 200000000))

        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[horizontal * self.eye_horizontal_multiplicator, vertical * self.eye_vertical_multiplicator],
                                                 time_from_start=duration)
        goal_msg.trajectory = JointTrajectory(joint_names=['eyes_shift_horizontal_joint', 'eyes_shift_vertical_joint'],
                                              points=[trajectory_points])

        self.eye_action_client.wait_for_server()

        self.eye_action_client.send_goal_async(goal_msg)
        # self.logger.info('eye location x: %f, eye location y: %f' % (horizontal, vertical))

    def send_horizontal_tilt_goal(self, horizontalTilt):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[-horizontalTilt, horizontalTilt], time_from_start=Duration(sec=1, nanosec=0))
        goal_msg.trajectory = JointTrajectory(joint_names=['head_tilt_left_joint', 'head_tilt_right_joint'],
                                              points=[trajectory_points])
        
        self.head_action_client.wait_for_server()

        self.head_action_client.send_goal_async(goal_msg)
        
    def send_pan_and_vertical_tilt_goal(self, pan, verticalTilt, duration=Duration(sec=0, nanosec=400000000)):
        # self.logger.info("Turning head to x: " + str(pan) + " y: " + str(verticalTilt))
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[pan * self.head_pan_multiplicator, verticalTilt * self.head_vertical_multiplicator], time_from_start=duration)
        goal_msg.trajectory = JointTrajectory(joint_names=['head_pan_joint', 'head_tilt_vertical_joint'],
                                              points=[trajectory_points])

        self.head_action_client.wait_for_server()

        self.head_action_client.send_goal_async(goal_msg)

    # Horizontal tilt is done separately and slower because the joints easily get stuck when moving quickly.
    def send_head_goal(self, pan, verticalTilt, horizontalTilt):
        #self.send_horizontal_tilt_goal(horizontalTilt) TODO: UNCOMMENT WHEN MECHANISM HAS BEEN MADE FUNCTIONAL AGAIN
        #time.sleep(1) # Wait a bit so that the new goal doesn't override the old, still-in-process goal
        self.send_pan_and_vertical_tilt_goal(pan, verticalTilt)

    """
    Calculates new pan and vertical tilt values corresponding to the face location coordinates given
    as arguments. 

    Returns: Absolute pan and vertical tilt values for head servos
    """
    def transform_face_location_to_head_values(self, face_location_x, face_location_y):
       
        # Calculate face movement
        x_diff = self.middle_x - face_location_x
        y_diff = self.middle_y - face_location_y

        # If the face is close enough to the center, leave the small movements for the eyes.
        if abs(x_diff) < 100:
            x_diff = 0
        if abs(y_diff) < 50:
            y_diff = 0

        # Transform face movement to head joint values
        # Head pan
        h_coeff = -0.00078

        """
        When eyes are used for movement, do not move head when there are multiple faces detected.
        Done to mitigate robot going back and forth between detected faces when they are roughly at the same distance.
        """
        if self.visible_face_amount > 1 and self.eyes_enabled:
            h_coeff = 0

        pan = x_diff * h_coeff + self.head_state[0]
        pan = max(min(self.head_pan_upper_limit, pan), self.head_pan_lower_limit) # limit head values to reasonable values
        # Alternative version: Adjust the pan value slightly to make smaller movements a bit bigger
        #pan = 0.8 * abs(x_diff * h_coeff) ** 0.8
        #pan = math.copysign(pan, -x_diff)

        # Vertical tilt
        v_coeff = -0.002
        vertical_tilt = y_diff * v_coeff + self.head_state[3]
        vertical_tilt = max(min(self.head_vertical_upper_limit, vertical_tilt), self.head_vertical_lower_limit)

        return pan, vertical_tilt

    """
    Calculates new x and y location for the eyes corresponding to the face location coordinates given
    as arguments.
    """
    def transform_face_location_to_eye_location(self, face_location_x, face_location_y):
        
        # Calculate face movement
        x_diff = self.middle_x - face_location_x
        y_diff = self.middle_y - face_location_y

        # Transform face movement to eye movement
        # Horizontal eye movement
        h_coeff = -0.002
        eye_location_x = x_diff * h_coeff + self.eyes_state[0]
        # Vertical eye movement
        v_coeff = 0.003
        eye_location_y = y_diff * v_coeff + self.eyes_state[1]
        
        eye_location_y = max(min(self.eye_vertical_upper_limit, eye_location_y), self.eye_vertical_lower_limit)
        eye_location_x = max(min(self.eye_horizontal_upper_limit, eye_location_x), self.eye_horizontal_lower_limit)

        return eye_location_x, eye_location_y

    #   Center eyes
    def center_eyes(self, duration=None):
        self.send_eye_goal(self.eyes_center_position[0], self.eyes_center_position[1], duration)


    """
    Returns a random location coordinates which is far enough from the current state of the eyes to be called a glance.
    """
    def get_random_eye_location(self, distance_from_current_x_position=0):
        # Get random x location for eyes
        random_x_list = list()
        if self.eye_horizontal_lower_limit < self.eyes_state[0] - distance_from_current_x_position:
            random_x_list.append(random.uniform(
                self.eye_horizontal_lower_limit, 
                self.eyes_state[0] - distance_from_current_x_position
                ))
        if self.eye_horizontal_upper_limit > self.eyes_state[0] + distance_from_current_x_position:
            random_x_list.append(random.uniform(
                self.eyes_state[0] + distance_from_current_x_position,
                self.eye_horizontal_upper_limit
                ))
        if len(random_x_list) == 0:
            self.logger.warning(f"Cannot get random eye position due to too large {distance_from_current_x_position =}" 
                                "compared to eye max and minimum locaitons! Setting random_x to current position.")
            random_x_list.append(self.eyes_state[0])
        random_x = random.choice(random_x_list)
        random_y = random.uniform(self.eye_vertical_lower_limit, self.eye_vertical_upper_limit)

        return random_x, random_y


def main(args=None):
    print('Hi from face_tracker_movement.')

    rclpy.init(args=args)

    action_client = FaceTrackerMovementNode()

    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
