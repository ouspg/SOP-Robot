# MovementGoal.msg - Defines a prioritized command to the MovementControllerNode

# Priority: 3 (GESTURE) > 2 (TRACK) > 1 (IDLE)
int32 priority 

# Target Joint Positions (Servo Angles)
float64 head_yaw_pan_target
float64 head_pitch_tilt_vertical_target
float64 eye_shift_horizontal_target
float64 eye_shift_vertical_target

# Duration for the movement
int64 duration_nanosecs