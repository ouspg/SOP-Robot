
This package contains a node that publishes image with the detected faces for debugging purposes and a message that contains location and landmarks of the detected faces. For the purpose of testing other functionality, such as eye movements, separately from face tracking, a mock face tracker node is also included.

![](./img/example.png)

## Parameters

face_tracker_node:

| Name             |                                    Description                                     |                               Default |
| ---------------- | :--------------------------------------------------------------------------------: | ------------------------------------: |
| lip_movement_detection |                      enable lip_movement_detection                           |                                  True |
| face_recognition |                                enable face_recognition                             |                                  True |
| correlation_tracking |                          enable correlation_tracking                           |                                  False |
| image_topic      |                                  Input rgb image                                   |                            /image_raw |
| image_face_topic | Output image with faces surrounded by triangles and face landmarks shown as circle |                            image_face |
| face_topic       |                Output face and face landmark positions in the frame                |   faces - face_tracker_msgs.msg.Faces |
| predictor        |                         Shape predictor data for landmarks                         | shape_predictor_68_face_landmarks.dat |
| lip_movement_detector |                            Lip_movement model                          | 1_32_False_True_0.25_lip_motion_net_model.h5 |


Webcam_node:

| Name             |                                    Description                                     |                               Default |
| ---------------- | :--------------------------------------------------------------------------------: | ------------------------------------: |
| raw_image        |                              Raw image output topic                                |                            /image_raw |
| index            |                          Device index, 0 for /dev/video0.                          |                                     0 |
| width            |                    Device width in pixels. Specify 0 for default.                  |                                     0 |
| height           |                    Device height in pixels. Specify 0 for default.                 |                                     0 |
| fps              |             Framerate. Specify 0 to publish at default (device) framerate          |                                     0 |
| mjpg             |                    Use mjpg compression, Specify False for default                 |                                 False |

Command `v4l2-ctl --list-formats-ext` can be used to determine, which webcam parameters can be used, if you are not satisfied with the default parameters. Using mjpg compression usually allows larger resolution and fps, but might lower image quality.
## Testing

The following launches the usb camera and face detector nodes. By default, it uses the first camera (`/dev/video0`).

```console
ros2 launch face_tracker face_tracker.test.launch.py
```

To view the camera feed, run: `ros2 run rqt_image_view rqt_image_view` and select the appropriate topic from the list.


The following launches the mock face tracker node. 
!!! The mock face tracker needs to be updated !!!

```console
ros2 run face_tracker mock_face_tracker_node
```
To use it, simply enter the desired coordinates on a single line, separated by either a comma or a space. The coordinates will then be published as a detected face location.


## Dependencies

(Not required for the mock face tracker)
* `Video4Linux2`
* `dlib`
* `opencv-python`
* `deepface`

## Potential future improvements

* Save recognized faces to some kind of database