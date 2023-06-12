## Overview
This package contains three nodes, **face_detection**, **emotion_detection** and **action_client**. Face_detection node detects a largest face from webcam image and sends the cropped image of the face to emotion_detection node which in turn runs inference and finds out the emotion on the face. Emotion_detection node then sends that emotion as a string to action_client node that controls a robot. Based on what emotion it received it send different actions.

## Building
```console
colcon build --packages-select emotion_detection
```

## Running
Source build packages.
```console
. install/local_setup.bash
```

The following command launches all three nodes.
```console
ros2 launch emotion_detection main.launch.py
```


## Dependencies

### Python
* `Tensorflow`
* `Opencv`
* `Numpy`

### Ros2
* `sensor_msgs`
* `std_msgs`
* `test_msgs`
* `cv_bridge`

## Troubleshooting
Remember to build in different terminal than where you source the installed package
