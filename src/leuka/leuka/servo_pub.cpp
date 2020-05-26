#include <ros/ros.h>
#include <std_msgs/UInt16.h>

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::UInt16>("servo", 10);

  ros::Rate loop_rate(5);

  int count = 0, var = 0;
  while (ros::ok())
  {

    std_msgs::UInt16 msg;
    if(count % 2 == 0){
	var = 1;
    }else{
        var = 0;
    }

    msg.data = var;

    ROS_INFO("%d", msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}
