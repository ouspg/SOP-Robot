# Getting started

The robot uses ROS (Robot Operating System). Read the [ROS wiki!](http://wiki.ros.org/).

## What is ROS? ([From the wiki](http://wiki.ros.org/ROS/Introduction))

ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. ROS is similar in some respects to 'robot frameworks,' such as Player, YARP, Orocos, CARMEN, Orca, MOOS, and Microsoft Robotics Studio.

The ROS runtime "graph" is a peer-to-peer network of processes (potentially distributed across machines) that are loosely coupled using the ROS communication infrastructure. ROS implements several different styles of communication, including synchronous RPC-style communication over services, asynchronous streaming of data over topics, and storage of data on a Parameter Server. These are explained in greater detail in our Conceptual Overview.

## Setup

TODO: add VM download link

VM username & password: ros

The VM comes with ROS melodic and [MoveIt](https://moveit.ros.org/) motion planning framework pre-installed.

Verify that `$WORKSPACE` defined in `.bashrc`
is set to the workspace directory.

Remember to `source ./devel/setup.sh` after `catkin_make` (`build.sh` does this)

## Project structure

* /src - all packages (3rd party and ours)
* /PMU2D2 - Arduino controller (just forwards data) for Dynamixel servos (is not 100% reliable)
* /config - Configuration files for servos, camera, etc

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
[ INFO] [1578781118.903485860]: Wait for scanning...
[ INFO] [1578781137.000997438]: Find 0 Dynamixels
[ INFO] [1578781137.004462202]: Succeed to init(115200)
[ INFO] [1578781137.004485675]: Wait for scanning...
[ INFO] [1578781154.744700503]: Find 0 Dynamixels
[ INFO] [1578781154.749514742]: Succeed to init(1000000)
[ INFO] [1578781154.749554728]: Wait for scanning...
[ INFO] [1578781172.191639197]: Find 0 Dynamixels
[ INFO] [1578781172.196411325]: Succeed to init(2000000)
[ INFO] [1578781172.196453934]: Wait for scanning...
[ INFO] [1578781189.600787542]: Find 0 Dynamixels
[ INFO] [1578781189.605546002]: Succeed to init(3000000)
[ INFO] [1578781189.605608034]: Wait for scanning...
[ INFO] [1578781206.979183036]: Find 0 Dynamixels
[ INFO] [1578781206.983838865]: Succeed to init(4000000)
[ INFO] [1578781206.983877742]: Wait for scanning...
[ INFO] [1578781224.336032479]: Find 0 Dynamixels
```

Commands can be send to servos using rqt when testing:

![](img/rqt_servo.PNG)

Address names are in `Camel_Case`.
Check the [dynamixel_item.cpp](https://github.com/ROBOTIS-GIT/dynamixel-workbench/blob/master/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_item.cpp) for full list of address names.

Note that servo communication packet timeout values were increased in [protocol2_packet_handler.cpp](src/DynamixelSDK/ros/src/dynamixel_sdk/protocol2_packet_handler.cpp)
to suit the Arduino. If you run into servo communication problems (such as `There is no status packet!`) you may want to change those.

```sh
roslaunch dynamixel-test.launch
```

TODO: add moveit support

Create the moveit config package as shown here:
http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html

