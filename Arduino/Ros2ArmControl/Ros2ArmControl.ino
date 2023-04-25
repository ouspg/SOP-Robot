#include <ros2arduino.h>
#include <Servo.h>

#ifndef LED_BUILTIN // To support some boards (eg. some esp32 boards)
#define LED_BUILTIN 13
#endif 

#define XRCEDDS_PORT  Serial 

Servo r_shoulder_lift;
Servo r_upper_arm_roll;
Servo r_elbow_flex;
Servo r_shoulder_out;

void subscribeArm(sensor_msgs::JointTrajectory* msg, void* arg)
{
  (void)(arg);
  r_shoulder_lift.write(msg.point[0])
  r_upper_arm_roll.write(msg.point[1])
  r_elbow_flex.write(msg.point[2])
  r_shoulder_out.write(msg.point[3])
}

class ArmSub : public ros2::Node
{
public:
  ArmSub()
  : Node("ros2arduino_sub_node")
  {
    this->createSubscriber<sensor_msgs::JointTrajectory>("/r_shoulder_controller/command", (ros2::CallbackFunc)subscribeArm, nullptr);
  }
};

void setup() 
{
  XRCEDDS_PORT.begin(115200);
  while (!XRCEDDS_PORT); 

  ros2::init(&XRCEDDS_PORT);
  r_shoulder_lift.attach(13)
  r_upper_arm_roll.attach(12)
  r_elbow_flex.attach(11)
  r_shoulder_out.attach(10)
}

void loop() 
{
  static ArmSub ArmNode;

  ros2::spin(&ArmNode);
}