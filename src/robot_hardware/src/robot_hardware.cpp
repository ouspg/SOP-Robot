#include "robot_hardware/robot_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

using std::placeholders::_1;

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

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

    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(info_.joints.size(), 0.0f);
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocity_.resize(info_.joints.size(), 0.0f);

    int idx = 0;

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {

      joint_indices_[joint.name] = idx++;

      /*if (joint.command_interfaces.size() != 1)
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
      }*/

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
      command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
              info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
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

        // hw_states_velocity_[joint_idx] = vel;
        hw_states_[joint_idx] = pos;

        /*RCLCPP_INFO(
            logger_,
            "Joint idx %d pos %.5f, vel %.5f", joint_idx, pos, vel);*/
      }
    }
  }

  bool RobotHardware::load_dynamixel_config(const std::string yaml_file)
  {
    YAML::Node cfg = YAML::LoadFile(yaml_file.c_str());
    if (cfg == NULL)
    {
      return false;
    }

    for (YAML::const_iterator it_file = cfg.begin(); it_file != cfg.end(); it_file++)
    {
      std::string name = it_file->first.as<std::string>();
      if (name.size() == 0)
      {
        continue;
      }

      YAML::Node item = cfg[name];
      for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
      {
        std::string item_name = it_item->first.as<std::string>();
        int32_t value = it_item->second.as<int32_t>();

        if (item_name == "ID")
        {
          dynamixel_[name] = value;
        }

        ItemValue item_value = {item_name, value};
        std::pair<std::string, ItemValue> info(name, item_value);

        dynamixel_info_.push_back(info);
      }
    }

    return true;
  }

  bool RobotHardware::arm_servos()
  {
    bool res = true;

    for (auto const &dxl : dynamixel_)
    {
      auto id = (uint8_t)dxl.second;
      if (!dxl_wb_.torqueOn(id))
      {
        RCLCPP_ERROR(logger_, "Could not arm servo: %d", id);
        res = false;
      }
    }

    return res;
  }

  bool RobotHardware::disarm_servos()
  {
    bool res = true;

    RCLCPP_INFO(logger_, "Disarm servos");

    for (auto const &dxl : dynamixel_)
    {
      auto id = (uint8_t)dxl.second;
      if (!dxl_wb_.torqueOff(id))
      {
        RCLCPP_ERROR(logger_, "Could not disarm servo: %d", id);
        res = false;
      }
    }

    return res;
  }

  // Check that servos are available and arm them
  bool RobotHardware::configure_dynamixels()
  {
    const char *log;

    for (auto const &dxl : dynamixel_)
    {
      uint16_t model_number = 0;
      if (!dxl_wb_.ping((uint8_t)dxl.second, &model_number, &log))
      {
        RCLCPP_ERROR(logger_, "%s", log);
        RCLCPP_ERROR(logger_, "Can't find Dynamixel ID '%d'", dxl.second);
        return false;
      }
      RCLCPP_INFO(logger_,
                  "Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);

      // Found servo, turn torque off
      dxl_wb_.torqueOff((uint8_t)dxl.second);

      // Configure servo
      for (auto const &info : dynamixel_info_)
      {
        if (dxl.first == info.first)
        {
          if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
          {
            if (!dxl_wb_.itemWrite(
                    (uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log))
            {
              RCLCPP_ERROR(logger_, "%s", log);
              RCLCPP_ERROR(logger_,
                           "Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]",
                           info.second.value, info.second.item_name.c_str(), dxl.first.c_str(),
                           dxl.second);
              return false;
            }
          }
        }
      }
    }
    return true;
  }

  bool RobotHardware::init_dynamixel_control_items()
  {
    auto it = dynamixel_.begin();

    const ControlItem *goal_position = dxl_wb_.getItemInfo(it->second, "Goal_Position");
    if (goal_position == NULL)
      return false;

    const ControlItem *goal_velocity = dxl_wb_.getItemInfo(it->second, "Goal_Velocity");
    if (goal_velocity == NULL)
      goal_velocity = dxl_wb_.getItemInfo(it->second, "Moving_Speed");
    if (goal_velocity == NULL)
      return false;

    const ControlItem *present_position = dxl_wb_.getItemInfo(it->second, "Present_Position");
    if (present_position == NULL)
      return false;

    const ControlItem *present_velocity = dxl_wb_.getItemInfo(it->second, "Present_Velocity");
    if (present_velocity == NULL)
      present_velocity = dxl_wb_.getItemInfo(it->second, "Present_Speed");
    if (present_velocity == NULL)
      return false;

    const ControlItem *present_current = dxl_wb_.getItemInfo(it->second, "Present_Current");
    if (present_current == NULL)
      present_current = dxl_wb_.getItemInfo(it->second, "Present_Load");
    if (present_current == NULL)
      return false;

    control_items_["Goal_Position"] = goal_position;
    control_items_["Goal_Velocity"] = goal_velocity;

    control_items_["Present_Position"] = present_position;
    control_items_["Present_Velocity"] = present_velocity;
    control_items_["Present_Current"] = present_current;

    return true;
  }

  bool RobotHardware::init_dynamixel_sdk_handlers()
  {
    bool result = false;
    const char *log = NULL;

    result = dxl_wb_.addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
    if (result == false)
    {
      printf("%s", log);
      return result;
    }
    else
    {
      printf("%s", log);
    }

    result = dxl_wb_.addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
    if (result == false)
    {
      printf("%s", log);
      return result;
    }
    else
    {
      printf("%s", log);
    }

    if (dxl_wb_.getProtocolVersion() == 2.0f)
    {
      uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

      /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */
      // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
      uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length + 2;

      result = dxl_wb_.addSyncReadHandler(start_address,
                                          read_length,
                                          &log);
      if (result == false)
      {
        printf("%s", log);
        return result;
      }
    }

    return result;
  }

  return_type RobotHardware::start()
  {
    RCLCPP_INFO(logger_, "Velocity control is not supported yet, so the trajectory is coarse")

    RCLCPP_INFO(
        logger_,
        "Starting ...please wait...");

    // Init dynamixel workbench
    // Todo: add port_name and baud_rate params
    if (!dxl_wb_.init())
    {
      RCLCPP_ERROR(logger_, "Could not initialize dynamixel workbench");
      return return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "Loading dynamixel config...");
    auto cfg_file = info_.hardware_parameters["dynamixel_info"];
    if (!load_dynamixel_config(cfg_file))
    {
      RCLCPP_ERROR(logger_, "Could not load dynamixel config from path: %s", cfg_file);
      return return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "Configuring dynamixels...");
    if (!configure_dynamixels())
    {
      return return_type::ERROR;
    }

    if (!init_dynamixel_control_items())
    {
      return return_type::ERROR;
    }

    if (!init_dynamixel_sdk_handlers())
    {
      return return_type::ERROR;
    }

    if (dxl_wb_.getProtocolVersion() != 2.0f)
    {
      RCLCPP_ERROR(logger_, "This hardware interface supports only Dynamixel protocol 2.0");
      return return_type::ERROR;
    }

    if (!set_default_servo_positions())
    {
      RCLCPP_ERROR(logger_, "Could not set default servo positions!");
      return return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "Arming servos...");
    if (!arm_servos())
    {
      RCLCPP_ERROR(logger_, "Could not arm servos");
      return return_type::ERROR;
    }

    // node_ = std::make_shared<rclcpp::Node>("robot_hardware");

    /*RCLCPP_INFO(logger_, "Subscribe to /joint_states");

    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&RobotHardware::joint_states_cb, this, _1));

    joint_traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory", 10);*/

    /*for (int i = 0; i <= hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      logger_,
      "%.1f seconds left...", hw_start_sec_ - i);
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

    if (!disarm_servos())
    {
      RCLCPP_ERROR(logger_, "Could not disarm servos!");
      return return_type::ERROR;
    }

    status_ = hardware_interface::status::STOPPED;

    RCLCPP_INFO(
        logger_,
        "System sucessfully stopped!");

    return return_type::OK;
  }

  bool RobotHardware::read_servo_values()
  {
    // rclcpp::spin_some(node_); // process queued work, e.g., process joint values callback

    bool result = false;
    const char *log = NULL;

    uint8_t id_array[dynamixel_.size()];
    uint8_t id_cnt = 0;

    // int32_t get_current[dynamixel_.size()];
    int32_t get_velocity[dynamixel_.size()];
    int32_t get_position[dynamixel_.size()];

    for (auto const &dxl : dynamixel_)
    {
      id_array[id_cnt++] = (uint8_t)dxl.second;
    }

    result = dxl_wb_.syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                              id_array,
                              dynamixel_.size(),
                              &log);
    if (!result)
    {
      RCLCPP_ERROR(logger_, "%s", log);
      return false;
    }

    // XL320 and XL430 servo models do not support this reading and it is not used anyway
    // effort is calculated from current
    /*result = dxl_wb_.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                     id_array,
                                     id_cnt,
                                     control_items_["Present_Current"]->address,
                                     control_items_["Present_Current"]->data_length,
                                     get_current,
                                     &log);
    if (!result)
    {
      RCLCPP_ERROR(logger_, "%s", log);
    }*/

    result = dxl_wb_.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                     id_array,
                                     id_cnt,
                                     control_items_["Present_Velocity"]->address,
                                     control_items_["Present_Velocity"]->data_length,
                                     get_velocity,
                                     &log);
    if (!result)
    {
      RCLCPP_ERROR(logger_, "%s", log);
      return false;
    }

    result = dxl_wb_.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                     id_array,
                                     id_cnt,
                                     control_items_["Present_Position"]->address,
                                     control_items_["Present_Position"]->data_length,
                                     get_position,
                                     &log);
    if (!result)
    {
      RCLCPP_ERROR(logger_, "%s", log);
      return false;
    }

    int idx = 0;
    for (auto const &dxl : dynamixel_)
    {
      int joint_idx = joint_indices_[dxl.first];
      hw_states_[joint_idx] = dxl_wb_.convertValue2Radian(dxl.second, get_position[idx]);
      hw_states_velocity_[joint_idx] = dxl_wb_.convertValue2Velocity(dxl.second, get_velocity[idx]);
      idx++;

      // RCLCPP_INFO(logger_, "Joint %s pos: %.3f, vel: %.3f", dxl.first.c_str(), hw_states_[joint_idx], hw_states_velocity_[joint_idx]);
    }
    return true;
  }

  bool RobotHardware::set_default_servo_positions()
  {
    if (!read_servo_values())
    {
      return false;
    }

    // Update values one by one in order to not invalidate references
    // If the hw_commands_ vector is replaced instead, the program crashes because the system has references to these `hw_commands_` values.
    for (uint i = 0; i < hw_commands_.size(); i++)
    {
      hw_commands_[i] = hw_states_[i];
    }
    return true;
  }

  hardware_interface::return_type RobotHardware::read()
  {
    return read_servo_values() ? return_type::OK : return_type::ERROR;
  }

  hardware_interface::return_type RobotHardware::write()
  {
    // auto msg = trajectory_msgs::msg::JointTrajectory();

    bool result = false;
    const char *log = NULL;

    uint8_t id_array[dynamixel_.size()];
    uint8_t id_cnt = 0;

    int32_t dynamixel_position[dynamixel_.size()];

    for (uint i = 0; i < hw_commands_.size(); i++)
    {
      auto joint_name = info_.joints[i].name;
      auto servo_id = dynamixel_.at(joint_name);
      id_array[id_cnt++] = servo_id;

      RCLCPP_INFO(logger_, "Move %s to pos %.3f, vel: %.3f", joint_name.c_str(), hw_commands_[i], hw_states_velocity_[i]);
    


      dynamixel_position[i] = dxl_wb_.convertRadian2Value(servo_id, hw_commands_[i]);

      // NaN == no work!
      /*if (std::isnan(hw_commands_[i]))
      {
        continue;
      }*/

      /*auto point = trajectory_msgs::msg::JointTrajectoryPoint();

      msg.joint_names.emplace_back(info_.joints[i].name);

      point.positions.emplace_back(hw_commands_[i]);
      point.velocities.emplace_back(.5f);
      // point.effort.emplace_back(.5f);
      point.accelerations.emplace_back(.2f);

      msg.points.emplace_back(point);*/

      // hw_commands_[i] = std::numeric_limits<double>::quiet_NaN();
    }

    /*result = dxl_wb_.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
                               id_array, id_cnt, dynamixel_position, 1, &log);
    if (!result)
    {
      RCLCPP_ERROR(logger_, "%s", log);
      return return_type::ERROR;
    }*/

    // Pub msg if it's not empty
    /*if (msg.joint_names.size() > 0)
    {
      joint_traj_pub_->publish(msg);
    }*/

    // rclcpp::spin_some(node_); // process queued work
    return return_type::OK;
  }
} // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    robot_hardware::RobotHardware,
    hardware_interface::SystemInterface)