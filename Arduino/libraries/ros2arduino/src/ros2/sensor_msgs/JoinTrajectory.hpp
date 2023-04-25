#ifndef _SENSOR_MSGS_JOINT_TRAJECTORY_HPP_
#define _SENSOR_MSGS_JOINT_TRAJECTORY_HPP_


#include "../topic.hpp"

#include "../std_msgs/Header.hpp"
#include "../sensor_msgs/JointTrajectoryPoint.hpp"

namespace sensor_msgs {

class JointTrajectory : public ros2::Topic<JointTrajectory>
{
public: 
    std_msgs::Header header;
    char joint_names[4][32];
    sensor_msgs::JointTrajectoryPoint point;

  JointTrajectory():
    Topic("sensor_msgs::msg::dds_::JointTrajectory_", "JointTrajectory", SENSOR_MSGS_JOINT_TRAJECTORY_ID),
    header(),
    joint_names_size(1),
    point()
  { 
    memset(joint_names, 0, sizeof(joint_names));
  }



  bool serialize(void* msg_buf, const JointState* topic)
  {
    ucdrBuffer* writer = (ucdrBuffer*)msg_buf;
    (void) header.serialize(writer, &topic->header);
    
    (void) ucdr_serialize_uint32_t(writer, topic->joint_names_size);
    for(uint32_t i = 0; i < topic->joint_names_size; i++)
    {
      (void) ucdr_serialize_string(writer, topic->joint_names[i]);
    } 

    (void) point.serialize(writer, &topic->point)

    return !writer->error;
  }

  bool deserialize(void* msg_buf, JointState* topic)
  {
    ucdrBuffer* reader = (ucdrBuffer*)msg_buf;
    uint32_t size_string;

    (void) header.deserialize(reader, &topic->header);
    
    (void) ucdr_deserialize_uint32_t(reader, &size_string);
    for(uint32_t i = 0; i < size_string; i++)
    {
      (void) ucdr_deserialize_string(reader, topic->joint_names[i], sizeof(topic->joint_names[i]));
    }

    (void) point.deserialize(reader, &topic->header)

  virtual uint32_t size_of_topic(const JointState* topic, uint32_t size)
  {
    uint32_t previousSize = size;

    size += header.size_of_topic(&topic->header, size);

    size += ucdr_alignment(size, 4) + 4;
    for(uint32_t i = 0; i < joint_names_size; i++)
    {
      size += ucdr_alignment(size, 4) + 4 + (uint32_t)(strlen(joint_names[i]) + 1);
    }

    size += point.size_of_topic(&topic->point, size)

    return size - previousSize;
  }
}
}
}// namespace sensor_msgs


#endif // _SENSOR_MSGS_JOINT_TRAJECTORY_HPP_