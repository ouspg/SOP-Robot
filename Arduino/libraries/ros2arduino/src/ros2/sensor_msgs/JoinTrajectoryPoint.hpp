#ifndef _SENSOR_MSGS_JOINT_TRAJECTORY_POINT_HPP_
#define _SENSOR_MSGS_JOINT_TRAJECTORY_POINT_HPP_


#include "../topic.hpp"

#include "../builtin_interfaces/Duration.hpp"
#include "../std_msgs/Float64.hpp"

namespace sensor_msgs {

class JointTrajectoryPoint : public ros2::Topic<JointTrajectoryPoint>
{
public: 

    std_msgs::Float64 positions[10];
    std_msgs::Float64 velocities[10];
    std_msgs::Float64 accelerations[10];
    std_msgs::Float64 effort[10];
    builtin_interfaces::Duration time_from_start;

  JointTrajectoryPoint():
    Topic("sensor_msgs::msg::dds_::JointTrajectoryPoint_", "JointTrajectoryPoint", SENSOR_MSGS_JOINT_TRAJECTORY_POINT_ID),
    positions(), velocities(), accelerations(), effort(), time_from_start()
  { 
  }



  bool serialize(void* msg_buf, const JointState* topic)
  {
    ucdrBuffer* writer = (ucdrBuffer*)msg_buf;
    (void) positions.serialize(writer, &topic->positions);
    (void) velocities.serialize(writer, &topic->velocities);
    (void) accelerations.serialize(writer, &topic->accelerations);
    (void) effort.serialize(writer, &topic->effort);
    (void) time_from_start.serialize(writer, &topic->time_from_start);

    return !writer->error;
  }

  bool deserialize(void* msg_buf, JointState* topic)
  {
    ucdrBuffer* reader = (ucdrBuffer*)msg_buf;
    (void) positions.deserialize(reader, &topic->positions);
    (void) velocities.deserialize(reader, &topic->velocities);
    (void) accelerations.deserialize(reader, &topic->accelerations);
    (void) effort.deserialize(reader, &topic->effort);
    (void) time_from_start.deserialize(reader, &topic->time_from_start);

  virtual uint32_t size_of_topic(const JointState* topic, uint32_t size)
  {
    uint32_t previousSize = size;

    size += positions.size_of_topic(&topic->positions, size)
    size += velocities.size_of_topic(&topic->velocities, size)
    size += accelerations.size_of_topic(&topic->accelerations, size)
    size += effort.size_of_topic(&topic->effort, size)
    size += time_from_start.size_of_topic(&topic->time_from_start, size)

    return size - previousSize;
  }
}
}
}// namespace sensor_msgs


#endif // _SENSOR_MSGS_JOINT_TRAJECTORY_POINT_HPP_