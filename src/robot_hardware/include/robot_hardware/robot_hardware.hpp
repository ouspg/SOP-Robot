#ifndef ROBOT_HARDWARE_HPP
#define ROBOT_HARDWARE_HPP

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include <yaml-cpp/yaml.h>



#include "rclcpp/rclcpp.hpp"
#include "visibility_control.h"

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

using hardware_interface::return_type;

namespace robot_hardware {

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

enum WriteHandlerType {
  Pos,
  Vel,
  Acc,
};

enum ControlItemType {
  DesiredPos,
  DesiredVel,
  DesiredAcc,
  PresentPos,
  PresentVel,
  PresentCurrent
};

typedef struct {
  int read_handler_id;
  std::unordered_map<WriteHandlerType, int> write_handlers;
  std::unordered_map<ControlItemType, const ControlItem*> write_control_items;
  std::map<std::string, uint32_t> servos;
} DynamixelGroup;


class RobotHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:

  RobotHardware();

  RCLCPP_SHARED_PTR_DEFINITIONS(RobotHardware)

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type start() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type stop() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type read() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type write() override;

private:
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_;
  std::vector<double> hw_states_velocity_;

  const rclcpp::Logger logger_;


  rclcpp::Node::SharedPtr node_;

  std::unordered_map<std::string, int> joint_indices_;

  DynamixelWorkbench dxl_wb_;

  std::vector<DynamixelGroup> dynamixel_groups_;

  std::map<std::string, uint32_t> dynamixel_; // all servos
  std::map<std::string, uint16_t> dynamixel_models_; // model numbers

  // std::map<std::string, uint32_t> dynamixel_;
  
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;

  bool set_default_servo_positions();
  bool read_servo_values();
  bool read_servo_group_values(const DynamixelGroup &group);

  bool write_servo_group_values(const DynamixelGroup &group);

  bool configure_dynamixels();
  bool load_dynamixel_config(const std::string yaml_file);
  bool init_dynamixel_control_items();
  bool init_dynamixel_sdk_handlers();

  bool arm_servos();
  bool disarm_servos();
};

}

#endif