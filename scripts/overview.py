from diagrams import Cluster, Diagram, Edge, Node
from diagrams.custom import Custom


class Box(Node):
    _provider = 'custom'
    _type = 'custom'
    _icon_dir = None

    fontcolor = '#ffffff'

    def _load_icon(self):
        return None

    def __init__(self, label):
        super().__init__(label)


with Diagram('High level overview of the robot', filename='img/overview', direction='TB'):
    u2d2 = Custom(icon_path='../img/u2d2.png', label='U2D2 (servo driver)')
    dynamixel = Custom(icon_path='../img/dynamixel.png', label='Servos')

    face = Custom(icon_path='../img/face.png', label='Camera')

    with Cluster('ROS nodes'):
        core = Box('ROS Core')
        servo_controller = Box('Dynamixel\nworkbench\n(servo controller)')
        face_detector = Box('Face Tracker')
        jaw_controller = Box('Jaw Controller')
        head = Box('Head gestures')

    # servo_controller >> Edge(label="publish /inmoov/joint_states") >> core
    _ = (
        servo_controller
        << Edge(label='sub /inmoov/joint_trajectory\npub /inmoov/joint_states')
        >> core
    )

    _ = jaw_controller >> Edge(label='pub /inmoov/joint_trajectory') >> core
    _ = head >> Edge(label='pub /inmoov/joint_trajectory\nsub /inmoov/head/gesture') << core

    _ = (
        servo_controller
        << Edge(label='USB')
        >> Edge(label='Control servos')
        >> u2d2
        << Edge()
        >> dynamixel
    )

    _ = face >> face_detector >> Edge(label='pub /inmoov/detection/faces') >> core
