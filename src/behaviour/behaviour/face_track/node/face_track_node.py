import math
import rclpy
from rclpy.node import Node

from face_tracker_msgs.msg import Faces, Face
from interface.msg import HeadMovementGoal
from core.util import run_node


class FaceTrackNode(Node):

    def __init__(self):
        super().__init__("face_track")

        self.declare_parameter("camera_diagonal_fov", 1.19555054)
        self.declare_parameter("camera_resolution_x", 1280)
        self.declare_parameter("camera_resolution_y", 960)
        self.declare_parameter("priority_face_track", 2)
        self.declare_parameter("tracking_goal_min_interval", 0.1)
        self.declare_parameter("head_movement_threshold", 0.15)

        camera_diagonal_fov = self.get_parameter("camera_diagonal_fov").value
        camera_resolution_x = self.get_parameter("camera_resolution_x").value
        camera_resolution_y = self.get_parameter("camera_resolution_y").value
        self.priority_face_track = self.get_parameter("priority_face_track").value
        self.tracking_goal_min_interval = self.get_parameter("tracking_goal_min_interval").value
        self.head_movement_threshold = self.get_parameter("head_movement_threshold").value

        self.angle_per_pixel = camera_diagonal_fov / math.sqrt(camera_resolution_x**2 + camera_resolution_y**2)
        self.middle_x = camera_resolution_x / 2
        self.middle_y = camera_resolution_y / 2

        self.goal_publisher = self.create_publisher(HeadMovementGoal, "/head_move_goal", 10)
        self.get_logger().info(f"Publisher [{self.goal_publisher.topic_name}] added.")

        self.create_subscription(Faces, "/faces", self.face_list_callback, 2)

        self.last_published_tracking_goal = 0.0
        self.get_logger().info("Node [face_track] initialized (i2head).")
        self.get_logger().info(
            "  Sub: /faces (face_tracker_msgs/Faces)"
        )
        self.get_logger().info(
            "  Pub: /head_move_goal (interface/msg/HeadMovementGoal)"
        )

    def __get_face_area(self, face: Face) -> float:
        width = face.bottom_right.x - face.top_left.x
        height = face.bottom_right.y - face.top_left.y
        return max(0.0, width * height)

    def __select_face_to_track(self, face_list: list[Face]):
        if not face_list:
            return None
        return max(face_list, key=self.__get_face_area)

    def face_list_callback(self, msg: Faces):
        current_time = self.get_clock().now().nanoseconds / 1e9

        if (current_time - self.last_published_tracking_goal) < self.tracking_goal_min_interval:
            return

        face_to_track = self.__select_face_to_track(msg.faces)

        if face_to_track is None:
            return

        width = face_to_track.bottom_right.x - face_to_track.top_left.x
        height = face_to_track.bottom_right.y - face_to_track.top_left.y

        center_x = face_to_track.top_left.x + (width / 2)
        center_y = face_to_track.top_left.y + (height / 2)

        pixel_error_x = center_x - self.middle_x
        pixel_error_y = center_y - self.middle_y

        angle_error_x = pixel_error_x * self.angle_per_pixel
        angle_error_y = pixel_error_y * self.angle_per_pixel

        head_pan_target = 0.0
        eye_horizontal_target = 0.0
        eye_vertical_target = 0.0

        if abs(angle_error_x) > self.head_movement_threshold:
            head_pan_target = angle_error_x
        else:
            eye_horizontal_target = angle_error_x

        eye_vertical_target = angle_error_y

        goal_msg = HeadMovementGoal()
        goal_msg.priority = self.priority_face_track
        goal_msg.head_yaw_pan_target = head_pan_target
        goal_msg.head_pitch_tilt_vertical_target = 0.0
        goal_msg.eye_shift_horizontal_target = eye_horizontal_target
        goal_msg.eye_shift_vertical_target = eye_vertical_target
        goal_msg.duration_nanosecs = 0

        self.get_logger().info(
            f"Head yaw= {goal_msg.head_yaw_pan_target:.4f} | "
            f"Eyes [hor= {goal_msg.eye_shift_horizontal_target:.4f}, "
            f"vert= {goal_msg.eye_shift_vertical_target:.4f}]"
        )

        self.goal_publisher.publish(goal_msg)
        self.last_published_tracking_goal = current_time


def main(args=None):
    run_node(FaceTrackNode)


if __name__ == "__main__":
    main()
