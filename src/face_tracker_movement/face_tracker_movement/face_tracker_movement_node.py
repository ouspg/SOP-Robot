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
            self.logger.info("Notice that the camera is not moved in simulation, when face is moveing. " + 
                             "This means that face location doesn't change, when head moves." +
                             "Idle movement should be similar, but has some hacky solutions that make it work.")

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

            self.simulation = True

        else:
            self.logger.info(f"Running face_tracker_movement in hardware mode")

            # Set initial values for actual robot
            # Movement limits
            self.head_vertical_lower_limit = 0.8
            self.head_vertical_upper_limit = 1.5
            self.head_pan_lower_limit = -0.75
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

            self.simulation = False

        self.camera_diagonal_fov = 1.19555054  # 60° In degrees
        self.camera_resolution_x = 1280
        self.camera_resolution_y = 960
        self.angle_per_pixel = self.camera_diagonal_fov / math.sqrt(self.camera_resolution_x**2 + self.camera_resolution_y**2)
        self.logger.info(f"{self.angle_per_pixel=}")

        # camera angle to eye and head servo angle coeffs (servo_angle = camera_angle * coeff)
        self.camera_angle_eye_vertical_coeff = 4.01489
        self.camera_angle_eye_horizontal_coeff = -2.67659
        self.camera_angle_head_vertical_coeff = -2.67659
        self.camera_angle_head_pan_coeff = -1.04387

        # Calculate camera angle constraints for eyes and head
        self.camera_angle_eye_vertical_lower_limit = self.eye_vertical_lower_limit / self.camera_angle_eye_vertical_coeff
        self.camera_angle_eye_vertical_upper_limit = self.eye_vertical_upper_limit / self.camera_angle_eye_vertical_coeff
        self.camera_angle_eye_horizontal_lower_limit = self.eye_horizontal_lower_limit / self.camera_angle_eye_horizontal_coeff
        self.camera_angle_eye_horizontal_upper_limit = self.eye_horizontal_upper_limit / self.camera_angle_eye_horizontal_coeff

        # Middle point of image view
        self.middle_x = self.camera_resolution_x / 2
        self.middle_y = self.camera_resolution_y / 2
        self.is_glancing = False
        self.idling = False # Used only for logging

        self.head_joint_ids = [4, 1, 3, 2]              # Servo ids for head joints. Order comes from head_controller: [head_pan_joint, head_tilt_right_joint, head_tilt_left_joint, head_tilt_vertical_joint]
        self.head_state = self.start_head_state[:]      # Tries to have the up-to-date head servo values.

        self.eyes_joint_ids = [9, 11]                   # Servo ids for eye joints. Order comes from eyes_controller: [eyes_shift_horizontal_joint, eyes_shift_vertical_joint]
        self.eyes_servo_state = self.eyes_center_position[:]      # Tries to have the up-to-date head servo values.
        self.eyes_state = [self.eyes_servo_state[0] / self.camera_angle_eye_horizontal_coeff, self.eyes_servo_state[1] / self.camera_angle_eye_vertical_coeff]

        # Some variables
        self.goal_pan = self.head_state[0]

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

        # TODO Propably not needed to update every time (as goal is 30fps). Consider moving this to own timer or something.
        self.update_blocked_faces()

        face = self.select_face_to_track(msg.faces)

        if face:
            # Reset idle callback
            self.idle_timer.timer_period_ns = 2000000000  # 2s
            self.idle_timer.reset()
            self.idling = False

            glance_percentage = 0.005 # Chance of executing a glance on each frame where a face has been detected. TODO: Decide on a good value
            # Check if doing the glance or not
            if random.uniform(0, 1) <= glance_percentage:
                self.glance()
                return  # Consider current head position old

            self.track_face(face)

        
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
                self.eyes_servo_state[i] = self.eyes_center_position[i]
                self.logger.info("Eye joint ID " + str(self.eyes_joint_ids[i]) + " is not responding")
            else:
                # Servo angle
                self.eyes_servo_state[i] = val
                # Camera angle
                self.eyes_state = [val[0] / self.camera_angle_eye_horizontal_coeff, val[1] / self.camera_angle_eye_vertical_coeff]

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
        elif gesture == 'glance':
            self.glance(**args)
        else:
            self.logger.info("Gesture not implemented!")

    # Get random horizontal positions for eyes and head and turn there at a random speed
    def idle_timer_callback(self):
        if self.idling == False:
            self.logger.info("Start idling...")
        self.idling = True

        # Only for simulator, gets around not getting servo positions 
        # this might have a bugged interaction for the idling movement after tracking!
        if self.simulation == True:
            self.head_state[0] = self.goal_pan

        # Max travel distance
        # TODO this should be a const calculated at the start!
        max_travel_distance = abs(self.head_pan_lower_limit) + abs(self.head_pan_upper_limit)

        max_idle_movement = 2 / 3 * max_travel_distance

        # Generate a random goal between the min and max pan limits
        # TODO: Consider adding larger possibility for no pan movement - only eyes move
        self.goal_pan = random.uniform(max(self.head_pan_lower_limit, self.head_state[0] - max_idle_movement), min(self.head_pan_upper_limit, self.head_state[0] + max_idle_movement))

        # Travel distance is current head state minus the position of the goal (e.g. current state is -0.2, random generated goal is -0.6. Travel distance is 0.4. abs(-0.2 - -0.4))
        # self.head_state[0] refers to the current state of the pan servo.
        pan_travel = self.head_state[0] - self.goal_pan
        travel_distance = abs(pan_travel)

        # Movement time is calculated based off the maximum travel distance available. Max pan from side to side is 4s, minimum movement time is 0.5s.
        # TODO tune this on real hardware to see what works! 
        movement_time = int(max(abs(travel_distance) / max_idle_movement * 3000000000, 750000000))

        # Commented for testing both head and eyes
        # if self.eyes_enabled:
            # self.send_eye_goal(self.get_random_eye_location()[0], self.eyes_center_position[1])

        self.logger.info(f"Eye midpoint is: {self.eyes_center_position[0]}")

        # Eyes will be biased to look in the direction of the head
        # TODO Normal or Uniform distribution for eyes going up/down?

        eye_goal_vertical = random.gauss(self.eyes_center_position[1], (self.eyes_center_position[1] + self.eye_vertical_upper_limit) / 3)

        if pan_travel > 0:
            eye_goal_horizontal = random.uniform(self.eye_horizontal_lower_limit, self.eye_horizontal_lower_limit / 6)
        elif pan_travel < 0:
            eye_goal_horizontal = random.uniform(self.eye_horizontal_upper_limit / 6, self.eye_horizontal_upper_limit)
        else:
            eye_goal_horizontal = random.uniform(self.eye_horizontal_lower_limit, self.eye_horizontal_upper_limit)

        # For testing only
        self.logger.info(f"{pan_travel=}")
        self.logger.info(f"{eye_goal_vertical=}")
        self.logger.info(f"{eye_goal_horizontal=}")

        if self.eyes_enabled:
            self.send_eye_goal(eye_goal_horizontal, eye_goal_vertical)

        if self.head_enabled:
            self.send_pan_and_vertical_tilt_goal(self.goal_pan, self.start_head_state[3], Duration(sec=0, nanosec = movement_time))
        # Reset idle timer to the length of movement + a random delay between 0.5s and 1.5s to make it feel more natural.
        # TODO fine-tune timer on real robot to see what fits.
        self.idle_timer.timer_period_ns = movement_time + random.uniform(750000000, 1500000000)
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

    def track_face(self, face):
        """
        Takes face message, calculates the head and eye movements from face location in pixels.
        Sends the movement goals for head and eyes.
        """
        pan_angle_head = 0
        vertical_angle_head = 0
        horizontal_angle_eyes = 0
        vertical_angle_eyes = 0

        # Calculate the face middle coordinates
        x=round((face.top_left.x + face.bottom_right.x) / 2)
        y=round((face.top_left.y + face.bottom_right.y) / 2)

        # Calculate movement in pixels
        x_diff = self.middle_x - x
        y_diff = self.middle_y - y

        # Convert pixel values to camera angle
        horizontal_angle = x_diff * self.angle_per_pixel
        vertical_angle = y_diff * self.angle_per_pixel

        if self.eyes_enabled and self.head_enabled:
            # Head and eyes are enabled - Divide the angle of total movement for head and eyes
            pan_angle_head, horizontal_angle_eyes = self.divide_horizontal_camera_angle_for_head_and_eyes(horizontal_angle)
            vertical_angle_head, vertical_angle_eyes = self.divide_vertical_camera_angle_for_head_and_eyes(vertical_angle)

        elif self.eyes_enabled:
            # Only eyes are enabled - move only eyes
            horizontal_angle_eyes = horizontal_angle
            vertical_angle_eyes = vertical_angle
        else:
            pan_angle_head = horizontal_angle
            vertical_angle_head = 0  # The vertical head movement is disabled

        # Transform camera angles to servo angles
        # Send movement goals
        if self.eyes_enabled and (horizontal_angle_eyes != 0 or vertical_angle_eyes != 0):
            eye_goal_horizontal, eye_goal_vertical = self.transform_camera_angle_to_eye_location(horizontal_angle_eyes, vertical_angle_eyes)
            self.send_eye_goal(eye_goal_horizontal, eye_goal_vertical)

        if self.head_enabled and (pan_angle_head != 0 or vertical_angle_head != 0):
            goal_pan, goal_vertical_tilt = self.transform_camera_angle_to_head_location(pan_angle_head, vertical_angle_head)
            self.send_pan_and_vertical_tilt_goal(goal_pan, goal_vertical_tilt) 

        # TODO memory of previous faces and look at their coordinates if you lose current face (in idle)

    def divide_horizontal_camera_angle_for_head_and_eyes(self, horizontal_angle):
        """
        Divide horizontal camera angle for eyes and head.
        If face is in center, but eyes are not centered, move eyes to center and head towards target horizontal angle.
        Otherwise take 1/3 of camera angle for eyes or move eyes to limit. Use remaining angle for head.
        """
        head_pan_angle = 0
        eyes_horizontal_angle = 0
        # Divide horizontal angle for head and eyes
        """ if abs(horizontal_angle) < math.pi / 36:  # head is in center sector of 10 degrees TODO: Adjust angle
            # if abs(self.eyes_state[0]) > abs(horizontal_angle):
            # Center the eyes over time
            # TODO The rate of centering can be dependent on face message frequency.
            eye_centering_multiplier = 0.2
            eyes_horizontal_angle = - self.eyes_state[0] * eye_centering_multiplier + horizontal_angle
            head_pan_angle = self.eyes_state[0] * eye_centering_multiplier
        else: """
        # Resolve eye movement - use 1/3 of total angle or move eyes to limit
        if horizontal_angle > 0:
            # Left movement
            eyes_horizontal_angle = min(self.camera_angle_eye_horizontal_lower_limit - self.eyes_state[0], horizontal_angle / 3)
        else:
            # Right movement
            eyes_horizontal_angle = max(self.camera_angle_eye_horizontal_upper_limit - self.eyes_state[0], horizontal_angle / 3)
        # Use the remaining for head
        # Applly movement limits later
        head_pan_angle = horizontal_angle - eyes_horizontal_angle
        return head_pan_angle, eyes_horizontal_angle
    
    def divide_vertical_camera_angle_for_head_and_eyes(self, vertical_angle):
        """
        Divide vertical camera angle for eyes and head.
        Currently vertical angle is not divided because head vertical movement is not working.
        TODO: Modify function when taking vertical movement into use
        """
        # vertical movement
        eyes_vertical_angle = vertical_angle
        head_vertical_angle = 0  # The vertical head movement is disabled
        return head_vertical_angle, eyes_vertical_angle

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

    def glance(self, delay=0.5, duration_of_individual_movements=None):
        """
        Makes a eye glance to random direction
        """
        if self.head_state:
            eye_location_horizontal, eye_location_vertical = self.get_random_eye_location(distance_from_current_horizontal_position = 0.5)
            self.send_eye_goal(eye_location_horizontal, eye_location_vertical)
            # Center the eyes back to the face after glancing
            time.sleep(delay)
            self.center_eyes(duration=duration_of_individual_movements)

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
        eye_x, eye_y = self.eyes_servo_state

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
            horizontal_diff = abs(self.eyes_servo_state[0] - horizontal)
            duration = Duration(sec=0, nanosec=max(int(150000000 * horizontal_diff), 150000000))

        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[horizontal * self.eye_horizontal_multiplicator, vertical * self.eye_vertical_multiplicator],
                                                 time_from_start=duration)
        goal_msg.trajectory = JointTrajectory(joint_names=['eyes_shift_horizontal_joint', 'eyes_shift_vertical_joint'],
                                              points=[trajectory_points])

        self.eye_action_client.wait_for_server()

        self.eye_action_client.send_goal_async(goal_msg)
        self.logger.info('eye location x: %f, eye location y: %f' % (horizontal, vertical))

    def send_horizontal_tilt_goal(self, horizontalTilt):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[-horizontalTilt, horizontalTilt], time_from_start=Duration(sec=1, nanosec=0))
        goal_msg.trajectory = JointTrajectory(joint_names=['head_tilt_left_joint', 'head_tilt_right_joint'],
                                              points=[trajectory_points])
        
        self.head_action_client.wait_for_server()

        self.head_action_client.send_goal_async(goal_msg)
        
    # TODO max time moved to 0.6s does this break anything?
    def send_pan_and_vertical_tilt_goal(self, pan, verticalTilt, duration=None):
        if duration == None:
            x_diff = abs(self.head_state[0] - pan)
            duration = Duration(sec=0, nanosec=int(200000000 * x_diff))
        self.logger.info("Turning head to x: " + str(pan) + " y: " + str(verticalTilt))
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


    def transform_camera_angle_to_eye_location(self, horizontal_angle, vertical_angle):
        """
        Calculate eye horizontal and vertical servo joint angles from camera angle
        Use limits if final location is outside movement limits.
        """
        
        # Transform camera angle to eye servo angle
        # Horizontal eye movement
        eye_location_x = horizontal_angle * self.camera_angle_eye_horizontal_coeff + self.eyes_servo_state[0]
        # Vertical eye movement
        eye_location_y = vertical_angle * self.camera_angle_eye_vertical_coeff + self.eyes_servo_state[1]
        
        eye_location_y = max(min(self.eye_vertical_upper_limit, eye_location_y), self.eye_vertical_lower_limit)
        eye_location_x = max(min(self.eye_horizontal_upper_limit, eye_location_x), self.eye_horizontal_lower_limit)

        return eye_location_x, eye_location_y

    def transform_camera_angle_to_head_location(self, horizontal_angle, vertical_angle):
        """
        Calculate head pan and vertical servo joint angles from camera angle.
        Use limits if final location is outside movement limits.
        """

        # Transform face movement to head joint values
        # Head pan
        pan = horizontal_angle * self.camera_angle_head_pan_coeff + self.head_state[0]
        pan = max(min(self.head_pan_upper_limit, pan), self.head_pan_lower_limit) # limit head values to reasonable values
        # Alternative version: Adjust the pan value slightly to make smaller movements a bit bigger
        #pan = 0.8 * abs(x_diff * h_coeff) ** 0.8
        #pan = math.copysign(pan, -x_diff)

        # Vertical tilt
        vertical_tilt = vertical_angle * self.camera_angle_eye_vertical_coeff + self.head_state[3]
        vertical_tilt = max(min(self.head_vertical_upper_limit, vertical_tilt), self.head_vertical_lower_limit)

        return pan, vertical_tilt

    #   Center eyes
    def center_eyes(self, duration=None):
        self.send_eye_goal(self.eyes_center_position[0], self.eyes_center_position[1], duration)

    """
    Returns a random location coordinates which is far enough from the current state of the eyes to be called a glance. Uses servo values.
    """
    def get_random_eye_location(self, distance_from_current_horizontal_position=0):
        # Get random x location for eyes
        random_horizontal_list = list()
        if self.eye_horizontal_lower_limit < self.eyes_servo_state[0] - distance_from_current_horizontal_position:
            random_horizontal_list.append(random.uniform(
                self.eye_horizontal_lower_limit, 
                self.eyes_servo_state[0] - distance_from_current_horizontal_position
                ))
        if self.eye_horizontal_upper_limit > self.eyes_servo_state[0] + distance_from_current_horizontal_position:
            random_horizontal_list.append(random.uniform(
                self.eyes_servo_state[0] + distance_from_current_horizontal_position,
                self.eye_horizontal_upper_limit
                ))
        if len(random_horizontal_list) == 0:
            self.logger.warning(f"Cannot get random eye position due to too large {distance_from_current_horizontal_position =}" 
                                "compared to eye max and minimum locaitons! Setting random_horizontal to current position.")
            random_horizontal_list.append(self.eyes_servo_state[0])
        random_horizontal = random.choice(random_horizontal_list)
        random_vertical = random.uniform(self.eye_vertical_lower_limit, self.eye_vertical_upper_limit)

        return random_horizontal, random_vertical


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
