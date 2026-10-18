# Unified arms

This ROS 2 package coordinates arm actions and hand gestures. It uses
`/dev/ttyACM0` for the physical arm controller and falls back to fake-hardware
behavior when that serial port is unavailable.

Run it directly with:

```console
pixi run unified-arms
```

The complete real and fake robot demos launch this node automatically.

## Simulated shoulders

The fake demos set `simulation:=true`, so the node never opens the Arduino
serial port. To run it separately with `pixi run robot-fake`:

```console
pixi run bash -c 'source install/local_setup.bash && ros2 run unified_arms unified_arms_node --ros-args -p simulation:=true'
```

Both arms support shoulder lift, upper-arm rotation, elbow flexion and wrist
rotation. The two shoulder-out joints are fixed at rest because the Arduino
has no channels for them. `wave`, `rock`, `rps_1` and `rps_2` use all six
physical arm servos. `zero` and `zer` request the firmware's startup
positions and reset the wrists, applying the normal command limits. The legacy
`hold` and `pos` commands target right shoulder lift and bicep (IDs 0 and 1),
leaving rotation unchanged.

The six Arduino channels use the firmware's integer angles and clamps:

| ID / pose index | Pin | Joint | Servo range | Startup angle | Commanded joint range |
| --- | --- | --- | --- | --- | --- |
| 0 | 3 | Right shoulder lift | 10–80° | 30° | −25–10° |
| 1 | 4 | Right elbow / bicep | 20–100° | 10° | −45–−5° |
| 2 | 5 | Right upper-arm rotation | 10–180° | 0° | −180–−10° |
| 3 | 8 | Left shoulder lift | 0–180° | 34° | −73–17° |
| 4 | 9 | Left elbow / bicep | 0–100° | 10° | −45–5° |
| 5 | 10 | Left upper-arm rotation | 0–180° | 80° | −80–100° |

IDs are array indices, not GPIO pin numbers. These values mirror `SERVO_PINS`,
`ServoMins`, `ServoMax`, and `expectedStartingPos` in the Arduino sketch.
The right bicep starts at 10° despite its 20° command minimum; right rotation
starts at 0° despite its 10° minimum. Firmware startup writes these values
directly, but subsequent commands are clamped. Simulation starts at the same
neutral pose and includes it in the URDF limits. Sending `zero` or `zer`
therefore commands 20° and 10° for those two servos (−5° bicep and −10° rotation).

Shoulder-out is disabled in the fake URDF, hardware interfaces, controllers
and gesture commands. Wrists use a
0–180° servo range centered on 90° (−90–90° joint rotation). At startup and
after a wrist reset, the right wrist uses a thumb-up neutral pose; the left
palm faces the user. Wrist rotation is measured from each hand's neutral pose,
with 90° of travel in each direction. The fake URDF applies a −90° right-hand
mounting offset and a 180° left-hand offset; joint angle 0 remains neutral.
Wrist controls remain available in simulation independently of the six-servo
Arduino firmware.

For example, `hold` commands 70° on both servos, producing −20° shoulder lift
and −30° elbow flexion. `pos` requests 45° lift and 120° bicep; the bicep
command is clamped to its firmware maximum of 100°, producing −7.5° shoulder
lift and −45° elbow flexion.

For a custom pose, send `pose:` followed by six servo angles in firmware order:
right lift, bicep, rotation, then left lift, bicep, rotation. Values 7 and 8
are reserved for the disabled shoulder-out joints and are ignored. Right
and left wrist angles remain values 9 and 10. Supported pose lengths are 6,
8, and 10 so existing poses keep their field order.
Use `-1` to leave an individual joint unchanged. Omitted extra axes retain
their previous positions. For example:

```console
pixi shell
source install/local_setup.bash
ros2 topic pub --once /arms/arm_action std_msgs/msg/String \
  '{data: "pose:50,50,75,64,60,110,0,0,110,65"}'
```

Prefix a preset or custom pose with `r_arm_` or `l_arm_` to select one arm,
for example `l_arm_rps_1` or `r_arm_pose:50,50,75,-1,-1,-1,0,0,110,-1`.
Real commands select the same physical joints, clamp angles, and send only
the selected IDs in ascending firmware order, terminated by a newline.
Simulation-only axes are omitted from Arduino commands.

Calibration lives in `inmoov_description/config/shoulder_simulation.yaml` and
also sets the fake URDF joint limits. Direction follows the existing URDF
servo mapping; neutral angles come from firmware startup positions. Shoulder
lift and bicep joints on both arms use an estimated 0.5 degrees of joint travel
per servo degree, based on the real robot moving about half as far as the
previous simulation. Other axes retain their existing scaling. Adjust
`joint_degrees_per_servo_degree` per joint when more precise measurements are
available. This scaling affects simulated motion and URDF limits; physical
servo commands retain the firmware angles. The fake robot starts in the
Arduino's power-on pose, before command clamping is applied.

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `simulation` | bool | `false` | Publish simulated shoulder trajectories without opening serial. A missing serial port also enables simulation. |
| `movement_duration` | double | `0.5` | Positive trajectory duration in seconds. This is adjustable animation timing; physical servo speed has not been measured. |

### ROS API

- Subscribes to `/arms/arm_action` (`std_msgs/String`).
- Publishes simulated commands to `/r_shoulder_controller/joint_trajectory`
  and `/l_shoulder_controller/joint_trajectory` (`trajectory_msgs/JointTrajectory`).
  Each controller accepts partial goals for its lift, rotation, elbow
  and wrist joints. Native trajectory positions are in radians;
  the `pose:` gesture input uses servo degrees.
- Publishes hand gestures to `/l_hand/l_hand_topic` and `/r_hand/r_hand_topic`
  (`std_msgs/String`).
- Publishes Arduino acknowledgements to `/feedback` (`std_msgs/String`).
