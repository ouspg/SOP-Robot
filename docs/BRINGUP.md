
**Note: Remember to always `colcon build` and `source install/setup.bash` first.**

## Bring-up fake (simulated) robot

The fake robot can be visualized using rviz. Run the following in a GUI environment:

```console
ros2 launch robot robot.fake.launch.py
```

This setups fake servo controllers and joint state publishers, and the following window should popup:

![](../img/inmoov_rviz.png)

**Note: currently, only jaw, eyes, right hand & head pan movement can be simulated**

**Todo: Add support for more joints**

## Bring-up real robot

Start with

```console
ros2 launch robot robot.launch.py
```
This should launch the robot listening server. If not, check the **servos not responding** part below.

Then open up another cli and do the following.

```console
ros2 control load_start_controller joint_state_broadcaster
ros2 control load_start_controller [YOUR CONTROLLER NAME HERE]
```
New way to start the services controllers without deprecated warning
```console
ros2 control load_controller --set-state start joint_state_broadcaster
ros2 control load_controller --set-state start r_hand_controller
```

Check if controllers were loaded and confirm that all controllers are in `active` state:

```console
ros2 control list_controllers
```

**Todo: simplify bring up process (add commands to launch file)**

## Test servo control

If the bring-up succeeded, the actions and topics should be available:

Example with head_controller
```console
vagrant@vagrant-ros:/workspace$ ros2 action list
/head_controller/follow_joint_trajectory

vagrant@vagrant-ros:/workspace$ ros2 topic list
/head_controller/joint_trajectory
/joint_states
```

For instance, if the joint `head_pan_joint` was configured correctly, it should move to position `0.5`
when publishing the following action (you are required to send positions for all joints):

```console
ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [head_pan_joint, head_tilt_right_joint, head_tilt_left_joint, head_tilt_vertical_joint],
    points: [
      { positions: [0.5, 0.0, 0.0, 0.0], time_from_start: { sec: 1, nanosec: 0 } }
    ]
  }
}"
```

`time_from_start` is the duration of the movement.

**Note: acceleration and velocity is fixed for real servos currently, so these cannot be controlled. This would require adding velocity and acceleration command interfaces to the JointTrajectoryAction controller**

## Servos are not responding?

Check that dynamixel.yaml has the right IDs and check that the baud rate of servos is set to 57600.

## Dynamixel ID X wasn't found?

All ID's that you have in the dynamixel.yaml need to be connected for the script to work. Comment out all servo ID's that are not connected. No need to remove them from anywhere else.

### But the servo(s) moved yet now doesn't move?!

The servo is likely overloaded. You have to manually reset the servo for it to work again. You can do this by using Dynamixel Wizard to reboot the servo, or alternatively you can turn the power off and on again. You will have to redo the whole bringup in any case.

## robot.launch.py claims that the hand has reached goal yet nothing happened

You have to enable the robot to use the actual parts and not fake simulated parts. This can be done in inmoov.urdf.xacro by setting the "fake" commands to false.
