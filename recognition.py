#Pirttiaho, Räty, Virtala 2020
import cv2
from math import *
import serial
import time
import numpy
import json
from tensorflow.keras import models

try:
    from helper_functions import *
except Exception as e:
    print("Failed to import helper_functions.py: {}".format(e))

class Recognition:
    faces = []
    latest_sphere_coordinates = []
    time_to_live = []
    id = -1

    def __init__(self, sp_coord, face, id):
        self.latest_sphere_coordinates = sp_coord
        self.faces.append(face)
        self.id = id
        self.reset_ttl()

    def tick_ttl(self):
        self.time_to_live -= 1
        #if self.time_to_live <= 0:
        #    del self

    def ttl(self):
        return self.ttl

    def reset_ttl(self):
        self.time_to_live = 10 #Set value for time to live

    def check_if_same(self, sp_coord, face, set):
        if set:
            r = sphere_distance(self.latest_sphere_coordinates, sp_coord)
            if r < 0.5: #in meters
                self.latest_sphere_coordinates = sp_coord
                self.faces.append(face)
                self.reset_ttl()
                print("New recognition")
                return True, self.time_to_live
        self.tick_ttl()
        return False, self.time_to_live


def write_text(image, text, position, scale, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    lineType = 2
    cv2.putText(image,text, position, font, scale, color, lineType)
    return image

def position_eyes(angle_x, angle_y):
    global eye_angles
    global disable_eye_movement

    if disable_eye_movement:
        return

    x_limits = [-30, 30] #-15, 15 Safe angle limits for the eyes
    y_limits = [-13.5, 8]

    angle_x = numpy.clip(angle_x, x_limits[0], x_limits[1])
    angle_y = numpy.clip(angle_y, y_limits[0], y_limits[1])

    #print(angle_x, angle_y)
    completed = False
    try:
        servo_x, servo_y = translate_to_servo_angles(angle_x, angle_y)
        completed = position_servos(servo_x, servo_y)
    except Exception as e:
        return e
    if completed:
        eye_angles = [angle_x, angle_y]
    return None

def translate_to_servo_angles(x, y):
    #Rotation multipliers defined by practical testing. Caused by undirect moving of the eyes using servos
    multipliers = [3, 4] #6, 7
    x_translated = (150 + (x * multipliers[0])) / (300 / 1024)
    y_translated = (150 + (-y * multipliers[1])) / (300 / 1024)

    return x_translated, y_translated

def position_servos(x, y):
    global ser #The serial session
    global cam_fov
    global disable_servos

    try:
        if not disable_servos:
            ser.flush()
            ser.reset_input_buffer()
            ser.reset_output_buffer()

        x_limits = [200, 820]
        y_limits = [400, 700]

        x = numpy.clip(x, x_limits[0], x_limits[1])
        y = numpy.clip(y, y_limits[0], y_limits[1])
        print(x, y)

        cmd_x = "<{},{},{}?>".format(1, 1, x).encode('ascii')
        cmd_y = "<{},{},{}?>".format(1, 2, y).encode('ascii')

        if not disable_servos:
            ser.write(cmd_x)
            ser.write(cmd_y)
        else:
            print("Servos driven to {}, {} digitally. Moving disabled".format(x, y))
    except Exception as e:
        print("Exception {} from moving the servos".format(e))
        return False
    else:
        return True

def servo_testing(): #Loop for testing the positioninng of servos according to userinput
    while True:
        try:
            sp = input("input angles: ").split(",")
            position_eyes(float(sp[0]), float(sp[1]))
        except Exception as e:
            print(e)

def add_recognition(coordinates, face, id):
    global recognitions
    is_same = False
    set = True
    for r in recognitions:
        if r:
            is_, ttl = r.check_if_same(coordinates, face, set)
            if ttl <= 0:
                recognitions.remove(r)
            if is_:
                is_same = True
                set = False

    if not is_same: #If not found create new Recognition class
        r_n = Recognition(coordinates, face, id)
        recognitions.append(r_n)

def recognition_thread(video_device_id, resolution, headless):
    global eye_angles
    global cam_fov
    global recognitions

    cap = cv2.VideoCapture(video_device_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

    classifier = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")# Create the haar cascade

    frame_i = 0
    while(True):
        print(len(recognitions))
        ret, frame = cap.read() #Read frame from the video device
        frame_i += 1
        if frame_i > 5: #Frame counter used to skip processing of frames. Used to help latency of detection
            frame_i = 0
        else:
            continue

        g_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = classifier.detectMultiScale(g_frame, scaleFactor = 1.3, minNeighbors = 5, minSize = (10, 10))
        size = frame.shape[:2]
        for index, (x, y, w, h) in enumerate(faces):
            cropped_face = frame[y:y + h,x:x + w] #Picture of the recognized face used to recognize person

            if not headless: #Draw detections to the frame
                cv2.rectangle(frame, (x, y), (x + w, y + h), [255, 255, 0], 2)
            center = get_center(x, y, h, w) #Define center of the image
            angles = get_angles(center[0], center[1], size[1], size[0], cam_fov) #Calculate the angles of the detections location
            distance = get_distance(w, h, size[1], size[0], cam_fov) #Approximate the distance of the detection

            add_recognition((angles[0], angles[1], distance), cropped_face, 0)
        if not headless:
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    if not headless:
        cap.release()
        cv2.destroyAllWindows()

def recognition_test(video_device_id, resolution, headless): #Follows random faces
    global eye_angles
    global cam_fov

    cap = cv2.VideoCapture(video_device_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

    colors = [[255, 255, 0], [0, 255, 0], [255, 0, 255]]
    classifier = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")# Create the haar cascade

    frame_i = 0
    while(True):
        ret, frame = cap.read() #Read frame from the video device
        frame_i += 1
        if frame_i > 5: #Frame counter used to skip processing of frames. Used to help latency of detection
            frame_i = 0
        else:
            continue

        g_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = classifier.detectMultiScale(g_frame, scaleFactor = 1.3, minNeighbors = 5, minSize = (10, 10))
        size = frame.shape[:2]
        for index, (x, y, w, h) in enumerate(faces):
            if index != 0:
                continue
            cropped_face = frame[y:y + h,x:x + w] #Picture of the recognized face used to recognize person

            if not headless: #Draw detections to the frame
                color = colors[numpy.clip(index, 0, len(colors) - 1)]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

            center = get_center(x, y, h, w) #Define center of the image
            #cv2.rectangle(frame, (center[0] - 2, center[1] - 2), (center[0] + 2, center[1] + 2), color, 2) #Code for drawing rectangle to highlight the center of detection
            angles = get_angles(center[0], center[1], size[1], size[0], cam_fov) #Calculate the angles of the detections location
            distance = get_distance(w, h, size[1], size[0], cam_fov) #Approximate the distance of the detection

            #print(round(angles[0],1), round(angles[1], 1), round(distance*100))
            #write_text(frame, "{}cm".format(round(distance * 100, 0)), (center[0], int(center[1] - h / 2)), 1, color)

            to_x, to_y = eye_angles[0] + angles[0], eye_angles[1] + angles[1]
            position_eyes(to_x, to_y) #Turn eyes to watch the wanted position
        if not headless:
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    if not headless:
        cap.release()
        cv2.destroyAllWindows()

def run_camera(video_device_id, resolution, fam_fov):
    global current_frame

    cap = cv2.VideoCapture(video_device_id) #Start capturing
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0]) #Set resolution of the capturing
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

    while(True):
        ret, current_frame = cap.read() #Read frame from the video device


#ROS INTERFACE
def detect_from_feed():
    global resolution
    global headless
    global classifier
    global detection_id

    if not current_frame:
        print("Camera not running. Cannot get frames")
        return drop_error_ros("Camera not running")
    frame = numpy.copy(current_frame) #Copy current frame to be used

    g_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Convert the frame to grayscale
    faces = classifier.detectMultiScale(g_frame, scaleFactor = 1.3, minNeighbors = 5, minSize = (10, 10)) #Classifier is given parameters defined in the writen work and returns detected faces

    size = frame.shape[:2] #Resolution of the frame
    ret = []
    for index, (x, y, w, h) in enumerate(faces):
        cropped_face = frame[y:y + h,x:x + w] #Picture of the recognized face used to recognize person

        if not headless: #Draw detections to the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), [255, 255, 0], 2)
        center = get_center(x, y, h, w) #Define center of the image
        angles = get_angles(center[0], center[1], size[1], size[0], cam_fov) #Calculate the angles of the detections location
        distance = get_distance(w, h, size[1], size[0], cam_fov) #Approximate the distance of the detection

        add_recognition((angles[0], angles[1], distance), cropped_face, id) #Add recognition to the list of recognized faces

        ret.append({"detection_id" : str(id), "pi_coord" : [str(center[0]), str(center[1])], "sp_coord" : [str(angles[0]), str(angles[1]), str(distance)]}) #Formulate the response for ros interfacing
        detection_id += 1
    return json.dumps(ret) #Return the create response to the client

def eyes(x, y): #Position eye ROS Function
    global eye_angles
    if x and y:
        ret = position_eyes(float(x), float(y))
        if ret:
            return drop_error_ros("Exception with eye movement: {}".format(ret))
    return json.dumps({"eye_position" : [str(eye_angles[0], str(eye_angles[1])]})

def encode(img): #Encode image to vector
    global encoder
    orig = [img]
    orig = numpy.expand_dims(orig, axis=-1)
    x_test = orig.astype("float32") / 255.0
    x_test = numpy.clip(x_test, 0, 1)
    out_vector = encoder.predict(x_test)[0]
    return out_vector

def recognize_person(): #ROS Function
    print("Recognition not yet implemented to ROS API")
    img = awdawd
    vector = encode(img)
    all_vectors = []
    for i in range(0, len(all_vectors)):
        vec = all_vectors[i]
        dist = calculate_distance(vector, vec, np.ones(32))
        if dist <= 600:
            return json.dumps({"person_id" : str(i)})
    return json.dumps({"person_id" : "-1"})

def drop_error_ros(message): #Quick function for formating ros compatible error message to return
    return json.dumps({"error" : str(message)})

#Settings
arduino_com = "/dev/ttyACM0" #CHANGE TO COM PORT WHEN USING WINDOWS
arduino_baud = 1000000 #115200 default. 1000000 used for greater throughput potential
video_device_id = 0 #ID of the web camera used
resolution = [640, 480] #Low resolution to speed up the processing

disable_eye_movement = False #Test variables to turn off features
disable_servos = False #DISABLES USAGE OF THE SERVOS

if disable_eye_movement:
    disable_servos = True

#Global variables
cam_fov = [53, 60] #Calculated Field of View of the MS HD 3000 web camera
eye_angles = [0, 0]
recognitions = [] #Monitored recognitions of the system
headless = not __name__ == "__main__"
detection_id = 0

current_frame = None
classifier = cv2.CascadeClassifier("haarcascade_frontalface_default.xml") # Load and create the classifier
encoder = models.load_model("encoder") #Load encoder from the trained model

#Initialize serial connection with the arduino
if not disable_servos:
    try:
        ser = serial.Serial(port = arduino_com, baudrate = arduino_baud)
    except Exception as e:
        if not headless:
            print("Failed to initialize serial connection to port {} at {}: {}".format(arduino_com, arduino_baud, e))

    if not headless:
        print("Starting serial communication")
    time.sleep(5) #Give serial session time to initialize
    position_servos(512, 512) #Reset servos to the middle


#run_camera(video_device_id, resolution, cam_fov)
#servo_testing()
#recognition_thread(video_device_id, resolution, headless)
#recognition_test(video_device_id, resolution, headless)
