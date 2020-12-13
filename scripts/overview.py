from diagrams import Diagram, Cluster, Edge, Node
from diagrams.aws.compute import EC2
from diagrams.aws.database import Aurora
from diagrams.aws.compute import Fargate
from diagrams.aws.general import Users
from diagrams.aws.network import ELB
from diagrams.aws.security import Cognito
from diagrams.onprem.container import Docker
from diagrams.onprem.compute import Server
from diagrams.oci.compute import Container
from diagrams.custom import Custom

class Box(Node):
    _provider = "custom"
    _type = "custom"
    _icon_dir = None

    fontcolor = "#ffffff"

    def _load_icon(self):
        return None

    def __init__(self, label):
        super().__init__(label)

with Diagram("High level overview of the robot", filename="img/overview", direction="TB"):
    
    
    u2d2 = Custom(icon_path="../img/u2d2.png", label="U2D2 (servo driver)")
    dynamixel = Custom(icon_path="../img/dynamixel.png", label="Servos")

    
    face = Custom(icon_path="../img/face.png", label="Camera")

    with Cluster("ROS nodes"):
        core = Box("ROS Core")
        servo_controller = Box("Dynamixel\nworkbench\n(servo controller)")
        face_detector = Box("Face Tracker")
        jaw_controller = Box("Jaw Controller")
        head = Box("Head gestures")

    # servo_controller >> Edge(label="publish /inmoov/joint_states") >> core
    servo_controller << Edge(label="sub /inmoov/joint_trajectory\npub /inmoov/joint_states") >> core

    jaw_controller >> Edge(label="pub /inmoov/joint_trajectory") >> core
    head >> Edge(label="pub /inmoov/joint_trajectory\nsub /inmoov/head/gesture") << core

    servo_controller << Edge(label="USB") >> Edge(label="Control servos") >> u2d2 << Edge() >> dynamixel

    face >> face_detector >> Edge(label="pub /inmoov/detection/faces") >> core


