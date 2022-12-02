# How to use simulation with face detection

Most of the requirements are listed in https://github.com/joelryt/SOP-Robot/blob/master/README.md.

This guide adds couple of important notes. 

**NOTE! Don't expect smooth operation with modest hardware. Running the ros2, simulation and face tracking uses a lot CPU.**

## Requirements for simulation

- You have some kind of webcam to use with face detection
- You are able to build and launch the robot with fake hardware
- You have installed ´python3-keras´ (At least on my environment it was not installed. Opencv_cam was unable to start.)

## Setup the Virtualbox

If you have a integrated webcam in you laptop you can use it. You need to pass the webcam hardware to Ubuntu guest from menu "Devices->Webcam->..." and choose your hardware. If you are using the usb connected external webcam configure it inside the USB settings. Or just pass it to guest with "Devices->USB->..." and choose your webcam.

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

## Launching the robot simulation

Now you should be able to launch the robot with fake hardware and use the face tracker.

```console
ros2 launch robot robot.fake.launch.py
```

Start the face tracker

```console
ros2 launch face_tracker face_tracker.test.launch.py
```

If you want to see what the face detection does, run this:

```console
ros2 run rqt_image_view rqt_image_view image_face_topic
```

To get the eyes moving. (Don't be scared about eyes flipping awkwardly to the side. More about this at the end...)

```console
ros2 run eye_movement eye_movement_node
```

Success! You are done. Eyes should "follow" your face. This implementation has a flaw that it is made for the real hardware, so the eye_movement node controls the eyes like it would have the real hardware. In other words, the node is tailored for working with the current hardware setup, not with the simulation.

Anyway, you are able to test the face tracking and eye movements like this.
