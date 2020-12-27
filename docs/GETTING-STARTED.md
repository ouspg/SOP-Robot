
## Servos

Dynamixel ROS instructions are [here](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/).

How to find connected dynamixel servos (serial port could be different for you):

```sh
$ rosrun dynamixel_workbench_controllers find_dynamixel /dev/ttyACM0
[ INFO] [1578781097.059273105]: Succeed to init(9600)
[ INFO] [1578781097.059380826]: Wait for scanning...
[ INFO] [1578781118.899728953]: Find 1 Dynamixels
[ INFO] [1578781118.899810859]: id : 1, model name : XL-320
[ INFO] [1578781118.903439606]: Succeed to init(57600)
```

```sh
roslaunch dynamixel-test.launch
```

Commands can be send to servos using rqt when testing:

![](img/rqt_servo.PNG)

Address names are in `Camel_Case`.
Check the [dynamixel_item.cpp](https://github.com/ROBOTIS-GIT/dynamixel-workbench/blob/master/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_item.cpp) for full list of address names.

Note that servo communication packet timeout values were increased in [protocol2_packet_handler.cpp](src/DynamixelSDK/ros/src/dynamixel_sdk/protocol2_packet_handler.cpp)
to suit the Arduino. If you run into servo communication problems (such as `There is no status packet!`) you may want to change those.
Feel free to improve the arduino controller; for example,
by using tri-state buffer (see [this](https://robottini.altervista.org/dynamixel-ax-12a-and-arduino-how-to-use-the-serial-port))

Now that we have verified that the servo can be controlled,
let's move to configuring and testing joints.
Joint communication happens using topics.
Dynamixel_workbench
publishes following topics:

```sh
$ rostopic list
/dynamixel_workbench/dynamixel_state
/inmoov/joint_trajectory
/joint_states
```

The type of the topic can be seen as follows:
```sh
$ rostopic type /inmoov/joint_trajectory
trajectory_msgs/JointTrajectory
```

The message structure can be seen using `rowmsg show` command:
```sh
$ rosmsg show trajectory_msgs/JointTrajectory
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string[] joint_names
trajectory_msgs/JointTrajectoryPoint[] points
  float64[] positions
  float64[] velocities
  float64[] accelerations
  float64[] effort
  duration time_from_start
```

Example joint config:

```yaml
head_pan_joint:
  ID: 1 # servo id
  Return_Delay_Time: 0
```

If the joint was configured correctly it should move to position 1
when publishing the following message:

```sh
rostopic pub -1 /inmoov/joint_trajectory trajectory_msgs/JointTrajectory -- '{header: auto, joint_names: ["head_pan_joint"], points: [{positions: [1], velocities: [1], accelerations: [1], effort: [1], time_from_start: 0}]}'
```

Read more about the YAML command line:
http://wiki.ros.org/ROS/YAMLCommandLine

## MoveIt!

![](./../img/moveit_pipeline.png)

https://moveit.ros.org/documentation/concepts/
