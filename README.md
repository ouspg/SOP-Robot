
# Getting started

The robot uses ROS2 (Robot Operating System 2). Read the [ROS2 wiki!](https://index.ros.org/doc/ros2/). Tutorials and how-to guides are worth to check. 

Read also [controller](https://control.ros.org/master/index.html) documentation to understand why certain controllers are needed and how they control the motion.
[Here](https://github.com/ros-controls/ros2_control_demos) are ros2 control demo repositories and more documentation about controllers.

## The Robot

The robot uses dynamixel servos. Servos are controlled via the `robot_hardware` ROS node, which implements the hardware interface for [ros2_control](https://github.com/ros-controls/ros2_control). `robot` package contains the servo controller configurations for the robot. See [robot.yaml](src/robot/controllers/robot.yaml), for example. The head supports `joint_trajectory_controller`. When using this controller, joints can be controlled by sending action messages to the trajectroy controller. See [bring up](./docs/BRINGUP.md) for an example of action message.

Joint -> Servo mappings are defined in two files. Configuration file connects servo ID with certain joint. [dynamixel_arm.yaml](config/dynamixel_arm.yaml) is for servos of robot arm and [dynamixel_head.yaml](config/dynamixel_head.yaml) for servos of head.

## Project structure

* client - contains client to send actions for robot hand
* config - dynamixel servo configuration
* src - all packages
  * src/arm_pack - not use in current configuration
  * src/eye_movement - runs action client to create eye motion commands based on the face tracking information
  * src/face_tracker - tracks the faces using camera feed and calculates locations and distances of faces and detects mouth movement. 
  * src/head_gestures - not use in current configuration
  * src/inmoov_description - robot files, which define the robot geometry and configuration for simulation (URDF, SRDF & rviz configuration)
  * src/robot - robot launch files & servo controller configurations
  * src/robot_hardware - hardware interface for ros2_controller, communicates with U2D2 via dynamixel workbench

## Servo Table

**Note: Servos can be configured using [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)**

Baud rate: 57600, Voltage: 7.5

**Note: You can use Dynamixel Wizard 2.0 to check that servos have enough voltage to operate normally.**

| Servo ID | Model | Joint name                  | Description           |
| -------- | ----- | --------------------------- | --------------------- |
| 1        | XL430 | head_tilt_right_joint       | Head tilt right-side  |
| 2        | XL430 | head_tilt_vertical_joint    | Head tilt up/down     |
| 3        | XL430 | head_tilt_left_joint        | Head tilt left-side   |
| 4        | XL430 | head_pan_joint              | Head turn left/right  |
| 9        | XL320 | eyes_shift_horizontal_joint | Eyes shift left/right |
| 11       | XL320 | eyes_shift_vertical_joint   | Eyes shift up/down    |
| 12       | XL320 | head_jaw_joint              | Open/close jaw        |
| 31       | XL320 | r_thumb_joint               | Open/close thumb      |
| 34       | XL320 | r_index1_joint              | Open/close index      |
| 37       | XL320 | r_middle1_joint             | Open/close middle     |
| 40       | XL320 | r_ring_joint                | Open/close ring       |
| 44       | XL320 | r_pinky_joint               | Open/close pinky      |
| 47       | XL320 | unnamed joint               | Rotate wrist          |

## Cautions
**Note: head tilt range of motion is poor**

**Note: wrist joint seems to overload very easily**

- Reason for this detected during inspection of the mechanical assembly. The hand is installed wrong way to the wrist.

**Note: servo angle limits are not configured**

**Note: Be careful not to move joints too much, limits are not set yet**

## Next steps

1. See [Development](./docs/DEVELOPMENT.md)
2. See [Robot bring-up](./docs/BRINGUP.md)
