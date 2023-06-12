# Python libraries
import cv2 # OpenCV library
import numpy as np
import sys
import time
import os

# ROS 2 packages
import rclpy 
from rclpy.node import Node 
from rclpy.executors import SingleThreadedExecutor
from rclpy.exceptions import ParameterNotDeclaredException
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory

# ROS 2 services/messages
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image

from .yunet import YuNet




class WebcamError(Exception):
    """signal that webcam has stopped working"""
    pass



class FaceDetection(Node):

    def __init__(self):
        super().__init__("face_recognition")
        self.logger = self.get_logger()
        self.logger.info("[*] Initializing face_recognition node...")

        self.frame_count = 0
        self.cv_bridge = CvBridge()
        self.cap = None
        self.open_webcam()
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

        # load Yunet model 
        package = os.path.realpath(__file__).split(os.sep)[-6] # get name of this package
        yunetPath = os.path.join(get_package_share_directory(package) + "/models/face_detection_yunet_2022mar.onnx")
        self.YUNET_model = YuNet(
            modelPath=yunetPath,
            inputSize=[320, 320],
            confThreshold=0.9,
            nmsThreshold=0.3,
            topK=5000,
            backendId=0,
            targetId=0
        )

        ret, frame = self.cap.read()
        if ret:
            height, width, _ = frame.shape
            self.YUNET_model.setInputSize((width, height))
        else:
            self.logger.fatal("[*] Failed to set Yunet input resolution")
            sys.exit(1)			

        # publisher for face image messages to emotion_detection node
        self._image_publisher = self.create_publisher(
            Image,						# message type
            "face_img",					# topic
            qos_profile=1,				# history depth
        )

        # timer to call face detection every 100ms
        self._pub_timer = self.create_timer(
            timer_period_sec=0.1,					# timer period in seconds
            callback=self.publish_faceImg_callback,	# callback function
        )
            
        # service to turn timer on/off
        self._timer_control_service = self.create_service(
            SetBool, 					 	# service type
            "timer_control", 			 	# service name
            self.timer_control_callback, 	# callback function
        )

        self.logger.info("[*] Initializing face_recognition node DONE!")



    def timer_control_callback(self, request, response):
        '''
        Callback that is called when service request is received.
        Change statemachine state to Stop or start timer that publishes images to topic "face_img".
        '''
        # if timer on
        if not self._pub_timer.is_canceled(): 
            if request.data == False:
                #self.logger.info("timer request -> close timer")
                response.success = True
                self._pub_timer.cancel() # turn off timer
                self.close_webcam()
            else:
                self.logger.warning("[*] timer request, call from a wrong state")
                response.success = False
            
        # if timer off
        else: 
            #self.logger.info("timer request -> start timer")
            if request.data == True:
                response.success = True
                self.open_webcam()
                self._pub_timer.reset() # start the timer again
            else:
                self.logger.warning("[*] timer request, call from wrong state")
                response.success = False

        return response


    def publish_faceImg_callback(self):
        '''
        Timer callback that publishes 48x48 grayscale image of detected face
        to 'face_img' topic. 
        '''
        
        try:
            # Capture a frame from webcam
            ret, frame = self.cap.read()
            if not ret:
                raise WebcamError

        except WebcamError:
            self.logger.error("[*] something went wrong, restarting webcam..")
            # close and try reopening webcam 
            self.close_webcam()
            self.open_webcam()

        else:
            self.frame_count += 1	
    
            # detect faces from image
            faces_list = self.YUNET_model.infer(frame)

            # check if any face was detected 
            if faces_list is not None:
                
                # find largest face from list of detected faces
                face_coords = self.find_largest_image(faces_list) # can maybe simplify check face_coordinates truth value
                if face_coords is None:
                    #Failed to find largest image, frame lost
                    return

                # crop 48x48 grayscale face out of original frame
                face = self.crop_face_image(frame, face_coords, padding=4)
                if face is None:
                    #Failed to crop face image, frame lost
                    return

                try:
                    image_msg = self.cv_bridge.cv2_to_imgmsg(face)

                except CvBridgeError:
                    self.logger.error("[*] CvBridgeError")

                else:
                    # publish image of face to topic
                    self._image_publisher.publish(image_msg)
                    self.logger.info(f'Publishing video frame: {self.frame_count}')	


    def open_webcam(self):
        '''
        Open webcam handle
        '''
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.logger.fatal("[*] Cannot open a webcam!")
            sys.exit(1)


    def close_webcam(self):
        '''
        Destroy webcam handle and close all windows
        '''
        #self.logger.info("closing webcam handle...")
        self.cap.release()
        cv2.destroyAllWindows()


    @staticmethod
    def crop_face_image(original_img, face_coords, padding=None):
        '''
        Crop face image from original_img using face_coords.
        if padding is not None, add padding to face coordinates to include more of the detected face
        for better emotion recognition.
        If no room for padding, return the original sized face image.
        :return: 48x48 grayscale image and [x, y, w, h] or None
        '''
        # unpack face coordinates from a tuple
        x, y, w, h = face_coords

        if padding is not None and padding > 0:
            # get array/image shape
            y_max, x_max, *_ = original_img.shape

            # check if padded coordinates are within bounds
            if (0 <= y-padding and y+h+padding < y_max and
            0 <= x-padding and x+w+padding < x_max): 
                x -= padding
                y -= padding
                h += padding
                w += padding

        # crop ROI from the webcam image
        face = original_img[y:y+h, x:x+w]

        try:
            # convert image to grayscale
            face_gray = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)
            #cv2_gray_img = cv2.equalizeHist(face_gray)
            # resize image to 48x48 (try cv2.INTER_AREA for better results but slower)
            face_gray = cv2.resize(face_gray, (48, 48), interpolation=cv2.INTER_NEAREST)
        except Exception as e:
            #print(str(e))
            return None
        else:
            return face_gray


    @staticmethod
    def find_largest_image(listof_face_coords):
        '''
        Find the largest image by calculating diagonal lenght for each rectangle and selecting the longest one.
        :return: [x, y, w, h] - coordinates of the largest face or None
        '''
        try:
            diag_lenghts = [np.linalg.norm([w, h], ord=2) for _, _, w, h, *_ in listof_face_coords]
            max_element_index = diag_lenghts.index(max(diag_lenghts))
        except ValueError:
            return None
        else:
            return list(map(int, listof_face_coords[max_element_index, :4]))





def main(args=None):
    # Initialize context
    rclpy.init(args=args) 

    try:
        executor = SingleThreadedExecutor()

        face_detection = FaceDetection()
        if executor.add_node(face_detection) == False:
            face_detection.get_logger().fatal("[*] Failed to add a node to the executor")
            sys.exit(1)

        try:
            while rclpy.ok():
                # fetch and execute new callbacks
                executor.spin_once()

        except KeyboardInterrupt:
            face_detection.get_logger().info("exiting...")

        finally:
            # unnecessary kinda
            executor.shutdown(timeout_sec=1)	
            face_detection.destroy_node()

    except Exception as e:
        print(str(e))

    finally:
        rclpy.shutdown() # will also shutdown global executor



if __name__ == '__main__':
    main()
