

The robot uses dynamixel servos. Servos are controlled via the `dynamixel workbench` ROS node. Therefore,
all nodes that wish to drive servos must publish message to the `/inmoov/joint_trajectory` topic. See the diagram below and the servo joint configuration: [config/dynamixel.yaml](./config/dynamixel.yaml)

![](img/overview.png)


See [Getting started](./docs/GETTING-STARTED.md)
