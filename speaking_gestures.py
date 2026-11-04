import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM = [
    "l_shoulder_lift_joint",
    "l_elbow_flex_joint",
    "l_upper_arm_roll_joint",
    "l_wrist_roll_joint"
]

HAND = [
    "l_thumb_joint",
    "l_index1_joint",
    "l_middle1_joint",
    "l_ring_joint",
    "l_pinky_joint"
]

HEAD = ["head_pan_joint"]

def send_trajectory(node, controller, joints, poses):
    client = ActionClient(
        node, FollowJointTrajectory,
        f"/{controller}/follow_joint_trajectory"
    )
    client.wait_for_server()

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joints

    for t, positions in poses:
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in positions]
        point.time_from_start = Duration(sec=t)
        goal.trajectory.points.append(point)

    return client, client.send_goal_async(goal)

def main():
    rclpy.init()
    node = Node("speaking_gestures")

    arm_poses = [
    	(1,  [0.0, 0.0, 0.0, -1.5]),
	(2,  [-0.3, -0.5, 0.3, -1.5]),
	(4,  [-0.4, -0.7, 0.5, -1.4]),
	(5,  [-0.25, -0.4, 0.1, -1.5]),
	(6,  [-0.4, -0.8, -0.2, -1.4]),
	(7,  [-0.3, -0.5, 0.4, -1.5]),
	(8,  [-0.2, -0.4, 0.0, -1.5]),
	(10, [-0.35, -0.65, 0.5, -1.4]),
	(11, [-0.2, -0.3, 0.2, -1.5]),
	(12, [0.0, 0.0, 0.0, -1.5])
    ]


    hand_poses = [
        (1,  [0.3, 0.3, 0.3, 0.3, 0.3]),
    	(2,  [0.1, 0.0, 0.0, 0.1, 0.1]),
    	(4,  [0.4, 0.6, 0.6, 0.7, 0.7]),
    	(5,  [0.2, 0.1, 0.1, 0.2, 0.3]),
    	(6,  [0.5, 0.8, 0.8, 0.9, 1.0]),
    	(7,  [0.2, 0.2, 0.2, 0.3, 0.3]),
    	(8,  [0.1, 0.0, 0.0, 0.1, 0.1]),
    	(10, [0.4, 0.6, 0.6, 0.7, 0.8]),
    	(11, [0.1, 0.1, 0.1, 0.2, 0.2]),
    	(12, [0.3, 0.3, 0.3, 0.3, 0.3])
    ]

    head_poses = [
        (1,  [0.0]),
    	(2,  [0.08]),
    	(4,  [0.12]),
    	(5,  [0.0]),
    	(6,  [-0.10]),
    	(7,  [-0.05]),
    	(8,  [0.05]),
    	(10, [0.10]),
    	(11, [0.0]),
    	(12, [0.0])
    ]

    commands = [
        ("l_shoulder_controller", ARM, arm_poses),
        ("l_hand_controller", HAND, hand_poses),
        ("head_controller", HEAD, head_poses)
    ]

    clients = []
    futures = []

    for controller, joints, poses in commands:
        client, future = send_trajectory(
            node, controller, joints, poses
        )
        clients.append(client)
        futures.append(future)

    handles = []
    for future in futures:
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("Liikekomento hylättiin")
        handles.append(handle)

    for handle in handles:
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future)
        result = future.result()
        if result.result.error_code != 0:
            raise RuntimeError(result.result.error_string)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

