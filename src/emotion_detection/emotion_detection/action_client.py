# Python packages
import time
import asyncio as aio
import sys

# ROS 2 packages
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# ROS 2 actions/messages
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ROS 2 services/messages
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from test_msgs.srv import BasicTypes




class MainActionClient(Node):

    def __init__(self):
        super().__init__('action_client')
        self.logger = self.get_logger()
        self.logger.info("[*] Initializing action_client node...")

        self.accepted_emotions = ("Angry", "Happy", "Sad", "Surprised")
        self.emotion = None
        self.should_reset = True
        # list to prevent garbage collectons from deleting task refrences
        self.background_tasks = []
        self.panServoAngle = 0.0

        self.start_timer_req = SetBool.Request(data=True)
        self.stop_timer_req = SetBool.Request(data=False)
        self.trigger_req = Trigger.Request()

        self.cb_group1 = ReentrantCallbackGroup()

        # action message constantsgit
        self.duration = Duration(sec=1,nanosec=0)
        self.head_joints = [
            'head_pan_joint'
        ]

        # only head pan movement can be simulated
        self.positions_dict = {
            "Original": [0.0 + self.panServoAngle],
            "Happy": [0.5 + self.panServoAngle], 		# look right
            "Angry":[-0.5 + self.panServoAngle], 		# look left
            "Sad": [-0.2 + self.panServoAngle], 		# look slightly left
            "Surprised": [0.2 + self.panServoAngle]	# look slightly right
        }

        # control robot movements
        self.action_client = ActionClient(
            self,				  						# node	
            FollowJointTrajectory, 						# action type
            '/head_controller/follow_joint_trajectory', # action name
            callback_group=self.cb_group1 				# callback group
        )

        # service to receive detected emotion "string"
        self.emotion_service = self.create_service(
            BasicTypes, 					# service type
            "detected_emotion",				# service name
            self.emotion_callback,			# callback functio	
            callback_group=self.cb_group1 	# callback group
        )
        
        # client to start or stop timer that publishes images to emotion_detection node
        self.timer_client = self.create_client(
            SetBool, 						# service type
            "timer_control",				# service name
            callback_group=self.cb_group1	# callback group
        )

        # start emotion node, to make it run inference again
        self.trigger_client = self.create_client(
            Trigger, 						# service type
            "start_emotion_node",			# service name
            callback_group=self.cb_group1 	# callback group
        )
        

        self.logger.info("[*] Initializing action_client node DONE!")

        

    async def emotion_callback(self, request, response):
        '''
        This callback is called when emotion is received.
        1.) Sends action goal to action server.
        2.) Stop timer in face_detection node
        '''
        
        # get the received emotion from received message 
        emotion = request.string_value
        #self.logger.info(f"received emotion: {emotion}")		

        # emotion in the ACCEPTED emotions list
        if emotion in self.accepted_emotions:
            self.logger.info("Emotion accepted -> proceeding...")

            self.should_reset = True
            response.bool_value = True

            # Turn off the timer in face_detection node
            timer_future = self.timer_client.call_async(self.stop_timer_req)

            # check if timer was successfully turned off
            timer_result = await timer_future
            if timer_result.success == True:
                self.logger.info("Timer turned off")
            else:
                self.logger.info("Failed to close timer")
        
            # start send_goal() task
            send_goal_task = aio.create_task(self.send_goal(emotion))
            self.background_tasks.append(send_goal_task)

        else:
            self.logger.info("Emotion denied -> exiting...")
            response.bool_value = False		

        return response


    async def send_goal(self, position):
        '''
        Send goal to robot and reset to original position after
        '''
        #self.logger.info("[*] send_goal called!, forming the goal msg")
        if position in self.positions_dict:
            # select goal position based on the emotion
            goal = self.positions_dict[position]
        else:
            self.logger.fatal("Received unknown emotion")
            sys.exit(1)

        # build a GOAL message 
        goal_msg = FollowJointTrajectory.Goal()
        jointTrajectoryPoint = [JointTrajectoryPoint(positions=goal, time_from_start=self.duration)]
        jointTrajectory = JointTrajectory(joint_names=self.head_joints, points=jointTrajectoryPoint)
        goal_msg.trajectory = jointTrajectory

        # check service availability (wait if not available)
        while not self.action_client.wait_for_server(timeout_sec=0.5):
            self.logger.info("robot action server not ready, waiting again...")
            await aio.sleep(0)

        #self.logger.info("Sending goal to robot...")
        goal_future = self.action_client.send_goal_async(
            goal_msg
        )
        goal_handle = await goal_future
        if not goal_handle.accepted:
            self.logger.info('Goal rejected :(')
            return
        self.logger.info('Goal accepted :)')

        res = await goal_handle.get_result_async()
        result = res.result
        #status = res.status
        if result.error_code == 0:
            self.logger.info("Goal succeeded!")
        else:
            self.logger.info("Goal failed...")
        
        if self.should_reset == True:
            self.should_reset = False
            time.sleep(2) # to not reset immediately
            # return back to start condition
            await self.ret_to_intial_state()	


    async def ret_to_intial_state(self):
        '''
        Resets this node to intial state.
        '''
        self.logger.info("Resetting robot to original position")
            
        # list to prevent garbage collectons from deleting task refrences
        self.background_tasks.clear()
        
        # reset robot to it's original position
        await self.send_goal("Original")

        self.logger.info("Resetting robot done!")


        # start running inference in emotion_detection node
        trigger_future = self.trigger_client.call_async(self.trigger_req)
        trigger_result = await trigger_future
        if trigger_result.success == False:
            self.logger.fatal("[*] Failed to start emotion node!")
            sys.exit(1)
        else:
            self.logger.info("Started emotion node!")

        # start timer in face_detection node
        timer_future = self.timer_client.call_async(self.start_timer_req)
        timer_result = await timer_future
        # make sure the timer has been started
        if timer_result == False:
            self.logger.fatal("[*] Failed to restart timer!")
            sys.exit(1)
        else:
            self.logger.info("Timer restarted!")
            self.start_time = time.time()





async def spin_async(executor):

    while rclpy.ok():
        # timeout_sec necessesary, otherwise blocks forever (or until a new callback arrives)
        executor.spin_once(timeout_sec=0.01) 
        # give control back to asyncio loop
        await aio.sleep(0) 


def main(args=None):
    # Initialize context
    rclpy.init(args=args) 

    try:
        executor = SingleThreadedExecutor()

        action_client = MainActionClient()
        if executor.add_node(action_client) == False:
            action_client.logger.fatal("[*] Failed to add a node to the executor")
            sys.exit(1)

        try:
            # fetch and execute new callbacks asynchronously
            aio.run(spin_async(executor=executor))

        except KeyboardInterrupt:
            action_client.logger.info("exiting...")

        finally:
            # unnecessary kinda
            executor.shutdown(timeout_sec=1)	
            action_client.destroy_node()
            
    except Exception as e:
        print(str(e))

    finally:
        rclpy.shutdown() # will also shutdown global executor



if __name__ == '__main__':
    main()
