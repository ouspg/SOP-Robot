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
      // Try pinging the servo
      if (!dxl_wb_.ping((uint8_t)dxl.second, &model_number, &log))
      {
        RCLCPP_ERROR(logger_, "Can't find Dynamixel ID '%d'", dxl.second);
        // If ping returns an error try rebooting the servo once

        RCLCPP_INFO(logger_, "Trying to reboot Dynamixel ID '%d'", dxl.second);
        dxl_wb_.reboot((uint8_t)dxl.second, &log);

        // Try pinging after rebooting
        if (!dxl_wb_.ping((uint8_t)dxl.second, &model_number, &log))
        {
          RCLCPP_ERROR(logger_, "Still can't find Dynamixel ID '%d'", dxl.second);
          RCLCPP_ERROR(logger_, "%s", log);
          RCLCPP_ERROR(logger_, "Skipping...", dxl.second);
          continue;
        }
      }
      RCLCPP_INFO(logger_,
                  "Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);

      dynamixel_models_[dxl.first] = model_number;

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
    std::map<uint16_t, DynamixelGroup> model_groups;

    for (const auto &dxl : dynamixel_)
    {
      const auto model_number = dynamixel_models_[dxl.first];

      if (model_number == 0) {
        continue;
      }

      // This model was already initialized
      if (model_groups.find(model_number) != model_groups.end())
      {
        auto &servos = model_groups[model_number].servos;
        servos[dxl.first] = dxl.second;
        continue;
      }

      DynamixelGroup group;
      group.servos[dxl.first] = dxl.second;

      RCLCPP_INFO(logger_, "Servo count: %d", group.servos.size());

      const ControlItem *goal_position = dxl_wb_.getItemInfo(dxl.second, "Goal_Position");
      group.write_control_items[ControlItemType::DesiredPos] = goal_position;

      const ControlItem *goal_velocity = dxl_wb_.getItemInfo(dxl.second, "Goal_Velocity");
      if (goal_velocity == NULL)
        goal_velocity = dxl_wb_.getItemInfo(dxl.second, "Moving_Speed");

      const ControlItem *profile_velocity = dxl_wb_.getItemInfo(dxl.second, "Profile_Velocity");
      group.write_control_items[ControlItemType::DesiredVel] = profile_velocity;

      const ControlItem *profile_acceleration = dxl_wb_.getItemInfo(dxl.second, "Profile_Acceleration");
      group.write_control_items[ControlItemType::DesiredAcc] = profile_acceleration;

      const ControlItem *present_position = dxl_wb_.getItemInfo(dxl.second, "Present_Position");
      group.write_control_items[ControlItemType::PresentPos] = present_position;

      const ControlItem *present_velocity = dxl_wb_.getItemInfo(dxl.second, "Present_Velocity");
      if (present_velocity == NULL)
        present_velocity = dxl_wb_.getItemInfo(dxl.second, "Present_Speed");
      group.write_control_items[ControlItemType::PresentVel] = present_velocity;

      const ControlItem *present_current = dxl_wb_.getItemInfo(dxl.second, "Present_Current");
      if (present_current == NULL)
        present_current = dxl_wb_.getItemInfo(dxl.second, "Present_Load");
      group.write_control_items[ControlItemType::PresentCurrent] = present_current;

      model_groups[model_number] = group;

      RCLCPP_INFO(logger_, "Initialized dynamixel model: %d", model_number);
    }

    for (const auto &pair : model_groups)
    {
      dynamixel_groups_.emplace_back(pair.second);
    }

    return true;
  }

  bool RobotHardware::init_dynamixel_sdk_handlers()
  {
    bool result = false;
    const char *log = NULL;

    int write_handler_id = 0;
    int read_handler_id = 0;

    for (auto &group : dynamixel_groups_)
    {
      const auto ctrl_items = group.write_control_items;

      const auto desired_pos = ctrl_items.at(ControlItemType::DesiredPos);
      const auto desired_vel = ctrl_items.at(ControlItemType::DesiredVel);
      const auto desired_acc = ctrl_items.at(ControlItemType::DesiredAcc);

      const auto present_pos = ctrl_items.at(ControlItemType::PresentPos);
      const auto present_vel = ctrl_items.at(ControlItemType::PresentVel);
      const auto present_curr = ctrl_items.at(ControlItemType::PresentCurrent);

      RCLCPP_INFO(logger_, "desired_pos: %p, desired_vel: %p, desired_acc: %p, present_pos: %p, present_vel: %p, present_current: %p",
                  desired_pos, desired_vel, desired_acc, present_pos, present_vel, present_curr);

      if (desired_pos != nullptr)
      {
        result = dxl_wb_.addSyncWriteHandler(desired_pos->address, desired_pos->data_length, &log);
        RCLCPP_INFO(logger_, "%s", log);

        if (result == false)
        {
          return result;
        }
        else
        {
          group.write_handlers[WriteHandlerType::Pos] = write_handler_id++;
        }
      }

      // result = dxl_wb_.addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);

      if (desired_vel != nullptr)
      {
        // Max velocity value
        result = dxl_wb_.addSyncWriteHandler(desired_vel->address, desired_vel->data_length, &log);
        RCLCPP_INFO(logger_, "%s", log);

        if (result == false)
        {
          return result;
        }
        else
        {
          group.write_handlers[WriteHandlerType::Vel] = write_handler_id++;
        }
      }

      if (desired_acc != nullptr)
      {
        result = dxl_wb_.addSyncWriteHandler(desired_acc->address, desired_acc->data_length, &log);
        RCLCPP_INFO(logger_, "%s", log);

        if (result == false)
        {
          return result;
        }
        else
        {
          group.write_handlers[WriteHandlerType::Acc] = write_handler_id++;
        }
      }

      uint16_t start_address = std::min(present_pos->address, present_curr->address);

      /*
        As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
      */
      // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
      uint16_t read_length = present_pos->data_length + present_vel->data_length + present_curr->data_length + 2;
      RCLCPP_INFO(logger_, "syncReadHandler read_length: %d", read_length);

      result = dxl_wb_.addSyncReadHandler(start_address,
                                          read_length,
                                          &log);
      if (result == false)
      {
        RCLCPP_ERROR(logger_, "%s", log);
        return result;
      }

      group.read_handler_id = read_handler_id++;
    }

    return result;
  }

  return_type RobotHardware::start()
  {
    RCLCPP_INFO(logger_, "Velocity control is not supported yet, so the trajectory is coarse");

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

    if (dxl_wb_.getProtocolVersion() != 2.0f)
    {
      RCLCPP_ERROR(logger_, "This hardware interface supports only Dynamixel protocol 2.0");
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

    if (!set_default_servo_positions())
    {
      RCLCPP_ERROR(logger_, "Could not set default servo positions!");
      return return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "Arming servos...");
    if (!arm_servos())
    {
      RCLCPP_ERROR(logger_, "Could not arm some servos");
    }

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

  bool RobotHardware::read_servo_group_values(const DynamixelGroup &group)
  {
    bool result = false;
    const char *log = NULL;

    const auto servos = group.servos;
    const auto ctrl_items = group.write_control_items;

    uint8_t id_array[servos.size()];
    uint8_t id_cnt = 0;

    // int32_t get_current[dynamixel_.size()];
    int32_t get_velocity[servos.size()];
    int32_t get_position[servos.size()];

    const auto present_pos = ctrl_items.at(ControlItemType::PresentPos);
    const auto present_vel = ctrl_items.at(ControlItemType::PresentVel);
    const auto present_curr = ctrl_items.at(ControlItemType::PresentCurrent);

    for (auto const &dxl : servos)
    {
      id_array[id_cnt++] = (uint8_t)dxl.second;
    }

    result = dxl_wb_.syncRead(group.read_handler_id,
                              id_array,
                              servos.size(),
                              &log);
    if (!result)
    {
      RCLCPP_ERROR(logger_, "syncRead [HandlerId: %d]: %s", group.read_handler_id, log);
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

    result = dxl_wb_.getSyncReadData(group.read_handler_id,
                                     id_array,
                                     id_cnt,
                                     present_vel->address,
                                     present_vel->data_length,
                                     get_velocity,
                                     &log);
    if (!result)
    {
      RCLCPP_ERROR(logger_, "Read velocity: %s", log);
      return false;
    }

    result = dxl_wb_.getSyncReadData(group.read_handler_id,
                                     id_array,
                                     id_cnt,
                                     present_pos->address,
                                     present_pos->data_length,
                                     get_position,
                                     &log);
    if (!result)
    {
      RCLCPP_ERROR(logger_, "Read position: %s", log);
      return false;
    }

    int idx = 0;
    for (auto const &dxl : servos)
    {
      // This servo does not have joint config, skip it
      // Todo: move this check to init code
      if (joint_indices_.find(dxl.first) == joint_indices_.end())
      {
        continue;
      }

      int joint_idx = joint_indices_[dxl.first];

      hw_states_[joint_idx] = dxl_wb_.convertValue2Radian(dxl.second, get_position[idx]);
      hw_states_velocity_[joint_idx] = dxl_wb_.convertValue2Velocity(dxl.second, get_velocity[idx]);
      idx++;

      // RCLCPP_INFO(logger_, "Joint %s pos: %.3f, vel: %.3f", dxl.first.c_str(), hw_states_[joint_idx], hw_states_velocity_[joint_idx]);
    }
    return true;
  }

  bool RobotHardware::read_servo_values()
  {
    bool all_success = true;

    for (auto const &group : dynamixel_groups_)
    {
      if (!read_servo_group_values(group))
      {
        all_success = false;
      }
    }
    return all_success;
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

  bool RobotHardware::write_servo_group_values(const DynamixelGroup &group)
  {
    bool result = false;
    const char *log = NULL;

    const auto servos = group.servos;

    uint8_t id_array[servos.size()];
    uint8_t id_cnt = 0;

    int32_t dynamixel_position[servos.size()];
    int32_t dynamixel_max_velocity[servos.size()];
    int32_t dynamixel_acceleration[servos.size()];

    for (const auto &servo : servos)
    {
      auto joint_name = servo.first;
      auto servo_id = servo.second;
      auto joint_idx = joint_indices_[joint_name];

      id_array[id_cnt] = servo_id;

      /*RCLCPP_INFO(logger_, "Move %s to pos %.3f, vel: %.3f", joint_name.c_str(),
                  hw_commands_[joint_idx], hw_states_velocity_[joint_idx]);*/

      // If cmd is NaN, use current position, i.e. do not move the servo
      if (std::isnan(hw_commands_[joint_idx]))
      {
        dynamixel_position[id_cnt] = dxl_wb_.convertRadian2Value(servo_id, hw_states_[joint_idx]);
        dynamixel_max_velocity[id_cnt] = 0;
        dynamixel_acceleration[id_cnt] = 0;
      }
      else
      {
        dynamixel_position[id_cnt] = dxl_wb_.convertRadian2Value(servo_id, hw_commands_[joint_idx]);
        dynamixel_max_velocity[id_cnt] = 200;
        dynamixel_acceleration[id_cnt] = 50;
      }

      id_cnt++;
    }

    // set pos if pos write handler exists
    if (group.write_handlers.find(WriteHandlerType::Pos) != group.write_handlers.end())
    {
      result = dxl_wb_.syncWrite(group.write_handlers.at(WriteHandlerType::Pos),
                                 id_array, id_cnt, dynamixel_position, 1, &log);
      if (!result)
      {
        RCLCPP_ERROR(logger_, "%s", log);
        return false;
      }
    }

    // set vel if vel write handler exists
    if (group.write_handlers.find(WriteHandlerType::Vel) != group.write_handlers.end())
    {
      result = dxl_wb_.syncWrite(group.write_handlers.at(WriteHandlerType::Vel),
                                 id_array, id_cnt, dynamixel_max_velocity, 1, &log);
      if (!result)
      {
        RCLCPP_ERROR(logger_, "%s", log);
        return false;
      }
    }

    // set acc if acc write handler exists
    if (group.write_handlers.find(WriteHandlerType::Acc) != group.write_handlers.end())
    {
      result = dxl_wb_.syncWrite(group.write_handlers.at(WriteHandlerType::Acc),
                                 id_array, id_cnt, dynamixel_acceleration, 1, &log);
      if (!result)
      {
        RCLCPP_ERROR(logger_, "%s", log);
        return false;
      }
    }

    return true;
  }

  hardware_interface::return_type RobotHardware::write()
  {
    bool all_success = true;

    for (auto const &group : dynamixel_groups_)
    {
      if (!write_servo_group_values(group))
      {
        all_success = false;
      }
    }

    return all_success ? return_type::OK : return_type::ERROR;
  }
} // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    robot_hardware::RobotHardware,
    hardware_interface::SystemInterface)