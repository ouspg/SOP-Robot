#ifndef ROBOT_HARDWARE_HPP
#define ROBOT_HARDWARE_HPP

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include <yaml-cpp/yaml.h>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "visibility_control.h"

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

using hardware_interface::return_type;

namespace robot_hardware {

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

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
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> hw_states_velocity_;

  const rclcpp::Logger logger_;


  rclcpp::Node::SharedPtr node_;

  std::unordered_map<std::string, int> joint_indices_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;

  DynamixelWorkbench dxl_wb_;

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem *> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;

  bool configure_dynamixels();
  bool load_dynamixel_config(const std::string yaml_file);
  bool init_dynamixel_control_items();
  bool init_dynamixel_sdk_handlers();

  bool arm_servos();
  bool disarm_servos();

  void joint_states_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}

#endif