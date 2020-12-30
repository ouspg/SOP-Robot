#include "robot_hardware/robot_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

using std::placeholders::_1;

namespace robot_hardware
{
  RobotHardware::RobotHardware() : logger_(rclcpp::get_logger("RobotHardware"))
  {
  }

  return_type RobotHardware::configure(
      const hardware_interface::HardwareInfo &info)
  {
    if (configure_default(info) != return_type::OK)
    {
      return return_type::ERROR;
    }

    hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    int idx = 0;

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {

      joint_indices_[joint.name] = idx++;

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            logger_,
            "Joint '%s' has %d command interfaces found. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return return_type::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            logger_,
            "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return return_type::ERROR;
      }

      /*if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' has %d state interface. 1 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }*/

      /*if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' have %s state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }*/
    }

    status_ = hardware_interface::status::CONFIGURED;
    return return_type::OK;
  }

  std::vector<hardware_interface::StateInterface>
  RobotHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));

      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  RobotHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  void RobotHardware::joint_states_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {

    for (int i = 0; i < msg->name.size(); i++)
    {

      /*RCLCPP_INFO(
          logger_,
          "Got %s state", msg->name[i].c_str());*/

      if (joint_indices_.find(msg->name[i]) != joint_indices_.end())
      {
        int joint_idx = joint_indices_.at(msg->name[i]);
        auto pos = msg->position[i];
        auto vel = msg->velocity[i];

        if (abs(pos) > M_PI)
        {
          RCLCPP_WARN(logger_,
                      "Something went wrong, received garbage joint pos reading: %0.5f", pos);
          continue;
        }

        hw_states_velocity_[joint_idx] = vel;
        hw_states_[joint_idx] = pos;

        /*RCLCPP_INFO(
            logger_,
            "Joint idx %d pos %.5f, vel %.5f", joint_idx, pos, vel);*/
      }
    }
  }

  return_type RobotHardware::start()
  {
    RCLCPP_INFO(
        logger_,
        "Starting ...please wait...");

    node_ = std::make_shared<rclcpp::Node>("robot_hardware");

    RCLCPP_INFO(logger_, "Subscribe to /joint_states");

    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&RobotHardware::joint_states_cb, this, _1));

    joint_traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory", 10);

    /*for (int i = 0; i <= hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      logger_,
      "%.1f seconds left...", hw_start_sec_ - i);
  }*/

    // set some default values
    /*for (uint i = 0; i < hw_states_.size(); i++) {
    if (std::isnan(hw_states_[i])) {
      hw_states_[i] = 0;
      hw_commands_[i] = 0;
    }
    if (std::isnan(hw_states_velocity_[i])) {
      hw_states_velocity_[i] = 0;
    }
  }*/

    status_ = hardware_interface::status::STARTED;

    RCLCPP_INFO(
        logger_,
        "System Sucessfully started!");

    return return_type::OK;
  }

  return_type RobotHardware::stop()
  {
    RCLCPP_INFO(
        logger_,
        "Stopping ...please wait...");

    /*for (int i = 0; i <= hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      logger_,
      "%.1f seconds left...", hw_stop_sec_ - i);
  }*/

    status_ = hardware_interface::status::STOPPED;

    RCLCPP_INFO(
        logger_,
        "System sucessfully stopped!");

    return return_type::OK;
  }

  hardware_interface::return_type RobotHardware::read()
  {
    rclcpp::spin_some(node_); // process queued work, e.g., process joint values callback
    return return_type::OK;
  }

  hardware_interface::return_type RobotHardware::write()
  {
    auto msg = trajectory_msgs::msg::JointTrajectory();

    for (uint i = 0; i < hw_commands_.size(); i++)
    {
      // NaN == no work!
      if (std::isnan(hw_commands_[i]))
      {
        continue;
      }

      auto point = trajectory_msgs::msg::JointTrajectoryPoint();

      msg.joint_names.emplace_back(info_.joints[i].name);

      point.positions.emplace_back(hw_commands_[i]);
      point.velocities.emplace_back(.5f);
      // point.effort.emplace_back(.5f);
      point.accelerations.emplace_back(.2f);

      msg.points.emplace_back(point);

      hw_commands_[i] = std::numeric_limits<double>::quiet_NaN();
    }

    // Pub msg if it's not empty
    if (msg.joint_names.size() > 0)
    {
      joint_traj_pub_->publish(msg);
    }

    rclcpp::spin_some(node_); // process queued work
    return return_type::OK;
  }
} // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    robot_hardware::RobotHardware,
    hardware_interface::SystemInterface)