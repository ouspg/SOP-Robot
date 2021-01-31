
This package contains a node that publishes image with the detected faces for debugging purposes and a message that contains location and landmarks of the detected faces.

![](./img/example.png)

## Parameters

| Name             |                                    Description                                     |                               Default |
| ---------------- | :--------------------------------------------------------------------------------: | ------------------------------------: |
| image_topic      |                                  Input rgb image                                   |                            /image_raw |
| image_face_topic | Output image with faces surrounded by triangles and face landmarks shown as circle |                            image_face |
| face_topic       |                Output face and face landmark positions in the frame                |   faces - face_tracker_msgs.msg.Faces |
| predictor        |                         Shape predictor data for landmarks                         | shape_predictor_68_face_landmarks.dat |


## Testing

The following launches the usb camera and face detector nodes. By default, it uses the first camera (`/dev/video0`).

```console
ros2 launch face_tracker face_tracker.test.launch.py
```

To view the camera feed, run: `ros2 run rqt_image_view rqt_image_view` and select the appropriate topic from the list.

## Dependencies

* `Video4Linux2`
* `dlib`
* [opencv_cam](https://github.com/clydemcqueen/opencv_cam)

These are included in the newest version of the vagrantfile

## Potential future improvements

* Use dlib correlation tracker or opencv boosting tracker to track faces across frames