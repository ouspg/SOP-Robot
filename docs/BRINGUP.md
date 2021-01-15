## Bring up real robot

**Note: Remember to `source setup/install.bash` first.**

```console
ros2 launch robot robot.launch.py
```

```console
ros2 control load_start_controller joint_state_controller
ros2 control load_configure_controller head_controller
```

Check if controllers were loaded:

```console
ros2 control list_controllers
```

Start controllers:

```console
ros2 control switch_controllers --start-controllers head_controller
```

Confirm that all controllers are in `active` state:

```console
ros2 control list_controllers
```

To echo joint states into the console:

```console
ros2 topic echo /joint_states
```

## Test servo control

For instance, If the joint `head_pan_joint` was configured correctly, it should move to position 1
when publishing the following action:

```console
ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [head_pan_joint],
    points: [
      { positions: [0.5], velocities: [0.1], accelerations: [0.1], time_from_start: { sec: 0, nanosec: 0 } }
    ]
  },
  goal_tolerance: [
    { name: head_pan_joint, position: 0.01 }
  ]
}"
```

**Note: goal tolerance checks seems to fail for now**

**Note: acceleration and velocity is fixed for servos currently, so these cannot be controlled**

<!--
## MoveIt!

![](./../img/moveit_pipeline.png)

https://moveit.ros.org/documentation/concepts/
-->
