# Python libraries
import numpy as np
import os
import time
from itertools import groupby
from collections import deque
from threading import Lock
import sys

# to disable tensorflow warning and info messages 
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' 

# different modules than on older version (using newest version, tensorflow v2.11 or sth)
from keras.models import load_model
from keras.utils import img_to_array

# ROS 2 packages
import rclpy 
from rclpy.node import Node 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ParameterNotDeclaredException
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory

# ROS 2 services/messages
from test_msgs.srv import BasicTypes
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image




class EmotionDetection(Node):

    def	__init__(self):
        super().__init__("emotion_detection")
        self.logger = self.get_logger()
        self.logger.info("[*] Initializing emotion_detection node...")

        self.cv_bridge = CvBridge()
        # circular buffer 
        self._buffsize = 3
        self.predicted_emotions = deque([], maxlen=self._buffsize) 
        
        self.emotions = ("Angry", "Fear", "Happy",
                            "Neutral", "Sad", "Surprised")
        
        self.emotion = None
        self.sending_emotion = False
        self.emotion_buffer_lock = Lock()		
        
        self.cb_group1 = ReentrantCallbackGroup()
        self.cb_group2 = MutuallyExclusiveCallbackGroup()
        self.cb_group3 = MutuallyExclusiveCallbackGroup()

        # load keras model 
        package = os.path.realpath(__file__).split(os.sep)[-6] # get name of this package
        self.FER_model = load_model(
            os.path.join(get_package_share_directory(package) + "/models/fer_2013.h5")
        )

        # subscriber receiving face images from face_detection node
        self._face_img_subscription = self.create_subscription(
            Image, 						  	# message type
            "face_img", 				  	# topic
            self.detect_emotion_callback, 	# callback function
            qos_profile=1,	  				# history depth
            callback_group=self.cb_group1	# callback group
        )
        
        # client to send detected emotion to action_client node
        self.action_client = self.create_client(
            BasicTypes, 					# service type
            "detected_emotion",				# service name
            callback_group=self.cb_group2 	# callback group
        )
        self.action_req = BasicTypes.Request()

        # service to start running inference again
        self.trigger_service = self.create_service(
            Trigger, 					 	# service type
            "start_emotion_node", 			# service name
            self.start_emotion_node, 		# callback function
            callback_group=self.cb_group3 	# callback group
        )

        self.logger.info("[*] Initializing emotion_detection node DONE!")



    def start_emotion_node(self, request, response):
        '''
        Start running inference again for received images
        '''
        #self.logger.info("[*] start_emotion_node called!")

        # to not skip running the inference!!
        if self.sending_emotion == True:
            self.sending_emotion = False
            response.success = True
        else:
            response.success = False
        return response


    async def detect_emotion_callback(self, image_msg: Image):
        '''
        Callback function that runs inference on received image to
        detect emotion.
        '''

        if self.sending_emotion == True:
            #EXIT before running inference
            return
        
        
        # convert image message back to image
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(image_msg)
        except CvBridgeError:
            self.logger.error("[*] CvBridgeError")
            return
          
        # run the inference
        pixels = img_to_array(cv_img)
        pixels = np.expand_dims(pixels, axis=0)
        predictions = self.FER_model.predict(pixels/255, verbose=0)
        index = np.argmax(predictions[0])
        
        # allow only 1 thread at a time to modify the circular buffer
        with self.emotion_buffer_lock:

            if self.sending_emotion == True:
                #EXIT after aquiring lock and doing nothing
                return 

            # add emotion into circular list/buffer
            self.predicted_emotions.append(self.emotions[index]) # atomic operation 

            # check that all (3) emotions in the buffer are the same to proceed
            if (self.all_elements_are_equal(self.predicted_emotions) and 
                len(self.predicted_emotions) >= self._buffsize and 
                not self.sending_emotion
            ):
                self.logger.info(f"predicted emotion: {self.emotions[index]}")
                self.sending_emotion = True	# stop other threads from doing uneccessary work (inference)
                self.emotion = self.emotions[index]
                self.predicted_emotions.clear()
                
            else:
                # exit callback if three consecutive emotions are not the same
                return
        
        # wait for action_service to be available
        while not self.action_client.wait_for_service(timeout_sec=0.5):
            self.logger.info("detected_emotion service not available, waiting again...")

        # send emotion to robot_action_client
        self.action_req.string_value = self.emotion # build request message
        future = self.action_client.call_async(self.action_req)
        result = await future

        # check if emotion was in the accepted list
        # otherwise keep sending emotions untill one is found
        if result.bool_value == False:
            self.logger.info("emotion declined!")
            self.sending_emotion = False	


    @staticmethod
    def all_elements_are_equal(iterable) -> bool:
        '''
        Returns True if all elements are the same in the iterable, otherwise returns false.
        '''
        group = groupby(iterable)
        return next(group, True) and not next(group, False)






def main(args=None):
    # Initialize context
    rclpy.init(args=args) 

    try:
        executor = MultiThreadedExecutor(num_threads=3) # try with 4 or leave empty so it uses 4 

        emotion_detection = EmotionDetection()
        if executor.add_node(emotion_detection) == False:
            emotion_detection.logger.fatal("[*] Failed to add a node to the executor")
            sys.exit(1)

        try:
            while rclpy.ok():
                # fetch and execute new callbacks
                executor.spin_once()

        except KeyboardInterrupt:
            emotion_detection.logger.info("CTRL + C, exiting...")

        finally:
            # unnecessary kinda
            executor.shutdown(timeout_sec=1)	
            emotion_detection.destroy_node()

    except Exception as e:
        print(str(e))
        
    finally:
        rclpy.shutdown() # will also shutdown global executor



if __name__ == '__main__':
    main()
