# Running the robot

Open the terminal in VM and build the robot code with `colcon build`. This will take a while especially on first run.

Next you need to source the built environment with `source install/setup.bash`. In short, you need to run this every time you have fresh build for your robot. This is set up in ~/.bashrc, so you don't need to run it in every new terminal before launching the robot or other node.

Now continue instructions in [Bring-up fake (simulated) robot][] or [Bring-up real HW robot][].

[Bring-up fake (simulated) robot]:#bring-up-fake-simulated-robot
[Bring-up real HW robot]:#bring-up-real-hw-robot

## Bring-up fake (simulated) robot

If you just want to test that robot starts, send hand written commands or use the [hand action client][] you don't need to fulfill any other requirement. However, if you want to test the face tracking you need have some webcam hardware. See [Webcam setup in Virtualbox][].

[Webcam setup in Virtualbox]:#webcam-setup-in-virtualbox

You can launch the fake robot in rviz using the launch file. Run the following command in a GUI environment:

```console
ros2 launch robot robot.fake.launch.py
```

This setups fake servo controllers and joint state publishers, and the following window should popup:

![](../img/inmoov_rviz.png)

Term ["fake hardware"][] means that the hardware mirrors received commands to its states. Ros2 Foxy documentation uses term "fake" but latest releases uses term "mock" talking about same thing.

### Launching the face tracker

If you want to use also the face tracker you need to start it:

```console
ros2 launch face_tracker face_tracker.test.launch.py
```

If you want to see what the face detection does, run this:

```console
ros2 run rqt_image_view rqt_image_view /face_tracker/image_face
```

This opens the view to see the camera feed and what face detection recognizes.

To get the eyes moving. (Don't be scared about eyes flipping awkwardly to the side. More about this at the end...)

```console
ros2 run eye_movement eye_movement_node
```

Success! You are done. Eyes should "follow" your face. This implementation has a flaw that it is made for the real hardware, so the eye_movement node controls the eyes like it would have the real hardware. In other words, the eyes have different "zero" position in simulation compared to real hardware.

Anyway, you are able to test the face tracking and eye movements like this.

## Bring-up real HW robot

### (0. Test servo communication)

We suggest using [Dynamixel Wizard 2.0][] to test the connection and functionality of the servos before trying to run the robot. This is optional but can save some time debugging if the servos or communication with them doesn't work.

### 1. Launching the robot

You can launch the real robot using a launch file:

```console
ros2 launch robot robot.launch.py
```

This should launch the robot listening server and prints a lot of output. If not, check the [Troubleshooting][] part below. `robot.launch.py` file creates temporary file from [dynamixel_arm.yaml][] and [dynamixel_head.yaml][] to `config/` folder to allow launching the arm and head separately.

[Troubleshooting]:#troubleshooting

To launch only the **arm** hardware

```console
ros2 launch robot robot.launch.py robot_parts:=arm
```

To launch only the **head** hardware

```console
ros2 launch robot robot.launch.py robot_parts:=head
```

### 1.5 Starting the controllers

In general you can start the controllers with:

```console
ros2 control load_controller --set-state start <controller_name>
```

### 2. Starting the controllers for hand
Open up another cli and start the controllers

```console
ros2 control load_controller --set-state start r_hand_controller
```

Check if controllers were loaded and confirm that all controllers are in `active` state:

```console
ros2 control list_controllers
```

Now you are ready to use the hand with the [hand action client][].

### 3. Starting the controller for eyes

If you completed the step 2. you can run this command in same cli. Otherwise open new cli.

Start the controller for eyes:

```console
ros2 control load_controller --set-state start eyes_controller
```

### 4. Launching the face tracker

After the required controllers are active, you can start the face tracker and eye movement nodes (face tracker can also be started before starting the controllers, eye movement node requires the controllers).

Start the face tracker

```console
ros2 launch face_tracker face_tracker.test.launch.py
```

If you want to see what the face detection does, run this:

```console
ros2 run rqt_image_view rqt_image_view /face_tracker/image_face
```

This opens the view to see the camera feed and what face detection recognizes.

Finally, start the eye movement node in a new terminal window

```console
ros2 run eye_movement eye_movement_node
```

**Todo: simplify bring up process (add the starting of the controllers to the launch file)**

## Sending action goals manually

The robot head joints doesn't have similar easy to use action client as the arm has. If you want to send action goals to the head you need to start the `head_controller` Then following actions and topics should be available:

```console
vagrant@vagrant-ros:/workspace$ ros2 action list
/head_controller/follow_joint_trajectory

vagrant@vagrant-ros:/workspace$ ros2 topic list
/head_controller/joint_trajectory
/joint_states
```

For example, if the joint `head_pan_joint` was configured correctly, it should move to position `0.5` when publishing the following action (other joints will also move to 0 positions if not already):

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

**Note: When driving the head_tilt_left/right joints, you must move the both servos simulatenously the same amount to the correct directions!**

**Note: The head_tilt_vertical_joint is easily overloaded due to the weight of the head and stickiness of the drive screw, and the servo will stop responding. Requires mechanical improvement.**

If you want to move only one joint at a time, it possible by omitting the other joints:

```console
ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [head_pan_joint],
    points: [
      { positions: [1.0], time_from_start: { sec: 2, nanosec: 0 } }
    ]
  }
}"
```

The jaw can be controlled with jaw_controller. Value for closed jaw is 0.0 and for open 0.55.

```console
ros2 action send_goal /jaw_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [head_jaw_joint],
    points: [
      { positions: [0.55], time_from_start: { sec: 1, nanosec: 0 } }
    ]
  }
}"
```

`time_from_start` is the duration of the movement.

**Note: acceleration and velocity is fixed for real servos currently, so these cannot be controlled. This would require adding velocity and acceleration command interfaces to the JointTrajectoryAction controller**

## Webcam setup in Virtualbox

If you have a integrated webcam in you laptop you can use it. You need to pass the webcam hardware to Ubuntu guest from menu "Devices->Webcam->..." and choose your hardware. If you are using the usb connected external webcam (e.g. the camera integrated to the robot eye) configure it inside the USB settings. Or just pass it to guest with "Devices->USB->..." and choose your webcam.

**If you have issues connecting the camera to guest make sure that you have the matching version of guest additions installed in the guest os.**

To test that OS detects the webcam run:

```console
lsusb -t
```

It should output something like this.

```console
vagrant@vagrant-ros:/workspace$ lsusb -t
/:  Bus 02.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/6p, 5000M
/:  Bus 01.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/8p, 480M
    |__ Port 1: Dev 3, If 0, Class=Video, Driver=uvcvideo, 12M
    |__ Port 1: Dev 3, If 1, Class=Video, Driver=uvcvideo, 12M
```

## Troubleshooting

### Dynamixel ID X wasn't found?

Check that [dynamixel_arm.yaml][] and [dynamixel_head.yaml][] have the right IDs and check that the baud rate of servos is set to 57600.

All ID's that you have in the dynamixel_*.yaml files need to be connected for the launch script to work. Comment out all servo ID's that are not connected. No need to remove them from anywhere else.

### But the servo(s) moved yet now doesn't move?!

The servo is likely overloaded. You have to manually reset the servo for it to work again. You can do this by using Dynamixel Wizard to reboot the servo, or alternatively you can turn the power off and on again. You will have to redo the whole bringup in any case.

### `colcon build` fails

There are various failures that we have seen so far, but here are some of the most common ones

#### 1. CMake clock skew error

CMake compiles the c++ packages and it does not like if the timestamps of temporary files are in in the future. This is most likely due to file synchronization between the host and guest OSes. It least we have seen this happening on Windows and Mac hosts. If you use Virtualbox with Vagrant it uses the Shared Folders interface to synchronize the /workspace folder.

You just need to try the `colcon build` build again. We have tried to make sure that the clocks on host and guest OS are synchronized without bulletproof results. Sometimes it helps you wait a while to compensate the time difference and run again `colcon build`.

Our suggestion is to test how environment works when folders are not synced from host OS. In other words move the whole development environment to guest OSes virtual disk. 

#### 2. Building just stops to random error

Please, run the `colcon build` again couple times, it might help. If you want to do some cleaning following commands have been sometimes helpful.

```console
colcon build --cmake-clean-cache
```

After you have run that try again `colcon build`.

#### 3. Build fails to dependency issue

If you face package dependency failures during `colcon build` please run `rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y`. It might request you to run `rosdep update` first. Run it first and then run the `rosdep install ...`. Then try to run `colcon build` again and it should be able to figure out those dependencies now.


<!-- References -->

[dynamixel_arm.yaml]:../config/dynamixel_arm.yaml
[dynamixel_head.yaml]:../config/dynamixel_head.yaml
[hand action client]:../client/README.md

[Dynamixel Wizard 2.0]: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/
["fake hardware"]: https://control.ros.org/galactic/doc/ros2_control/hardware_interface/doc/fake_components_userdoc.html
