# HAND CLIENT

## Shoulder movement tester

Run:

```console
pixi run python client/shoulder_tester.py
```

move shoulder servos from -90 to 90 degrees from servo's zero position using index:degree,index:degree syntax.

The current index's are

| Index   | Servo function |
| ----- | -----------------------   | 
|   0   | R shoulder lift | 
|   1   | R upper arm rotation | 
|   2   | R bicep           | 
|   3   | R shoulder out   | 
|   4   | L shoulder lift | 
|   5   | L upper arm rotation | 
|   6    | L bicep | 
|   7    | L shoulder out | 

Example: 

  ```
  shoulders> 0:10, 1:-15
  ```
  moves R shoulder lift servo 10 degrees to the positive direction and R upper arm rotation servo 15 degrees to the negative direction.



## How to run

Ensure that you have set up the robot as instructed in [Robot bring-up](../docs/BRINGUP.md) and that r_hand_controller is up and running. If not, refer back to the [bring-up](../docs/BRINGUP.md).

Once the robot is running properly, open a new command window, and run the hand_gestures_node with the following command.

### The simulator only supports finger movements. The unified_arms node tries to connect over serial and assumes fake hardware when the port is unavailable.
```
ros2 run hand_gestures hand_gestures_node
```
Then run the unified arms node in a new terminal window:

```
ros2 run unified_arms unified_arms_node
```

Run tester client in new terminal window, from there you can send commands to hands
```
python3 client/hand_client_tester.py
```

## How to use

Once the client is up and running you will be greeted by a command line interface asking for your next command.

```
Input command:
```

Type in the command you want and watch as the hands completes the action. 
### Note: The simulator only support the fingers movements in both hands. It does not support hand movements.

## The commands

Currently, the following actions are available.

| Action    | What it does                              |
| --------  | ----------------------------------------- | 
| wave      | Waves with the left hand                  | 
| rock      | Rocks with the left hand                  | 
| test      | Tests the motion of both hands            | 
| zero      | Puts both hands at resting position with  | 

## Fingers-only movements

| Action    | What it does                              |
| --------  | ----------------------------------------- | 
| open      | Extends all fingers                       | 
| fist      | Forms a fist                              | 
| scissors  | Extends index and middle finger           | 
| point     | Extends index finger                      | 
| thumbs_up | Gives a thumbs up!                        | 
| grasp     | Grasps an object                          | 
| hard_rock | Heavy metal                               | 
| pen_grasp | Grasps with index and middle finger       | 
| rps       | Plays a round of Rock-Paper_scissors      | 

## IMPORTANT THE CUSTOM STATES MUST BE BETWEEN -0.5 AND 2 YOU WILL BREAK THE HAND IF YOU GO BEYOND THESE

# HEAD GESTURE CLIENT

## How to run

Ensure that you have set up the robot as instructed in [Robot bring-up](../docs/BRINGUP.md). face_tracker_movement_node also needs to be started up, which can be done with the following command:

```
ros2 run face_tracker_movement face_tracker_movement_node
```

Once the robot is running properly, open a new command window, and start the client with the following command.

```
python3 client/head_gesture_client.py 
```

## How to use

Once the client is up and running you will be greeted by a command line interface asking for your next command.

```
Input command:
```

Type in the command you want and watch as the head completes the action.

## The commands

Currently, the following actions are available.

| Gesture   | What it does                              |
| --------  | ----------------------------------------- | 
| nod       | Nods                                      | 
| shake     | Shakes head                               | 

Optionally, the following arguments are also available for the currently implemented gestures.

| Gesture   | What it does                                                               |
| --------  | -------------------------------------------------------------------------- | 
| magnitude | Sets the magnitude of the head movements in the gesture                    | 
| delay     | Sets the delay between the beginning of each head movement in the gesture  |
| duration  | Sets the duration of the individual head movements in the gesture          |

Arguments can be used by adding them after the command, separated by commas in the following format

```
command, argument1_name=argument1_value, argument2_name=argument2_value
```

Example commands:

```
nod
```
```
shake, magnitude=0.5, delay=0.5, duration=0.4
```
```
nod, duration=0.4
```

The client does not check argument validity, but invalid arguments are ignored by the face_gestures_node. The 'quit' and 'exit' commands do not take arguments.
