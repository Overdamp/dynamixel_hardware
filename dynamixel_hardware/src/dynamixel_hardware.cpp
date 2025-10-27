// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
// Modified to force Velocity Control Mode for Mobile Base

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>
#include <cmath> // For std::isnan

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";

return_type DynamixelHardware::configure(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("use_dummy") == "true") {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
    status_ = hardware_interface::status::CONFIGURED;
    return return_type::OK;
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate);

  if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return return_type::ERROR;
  }

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint16_t model_number = 0;
    if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return return_type::ERROR;
    }
  }

  enable_torque(false);
  
  // --- [CUSTOM FIX] ---
  // Force Velocity Control Mode instead of Position Mode for Mobile Base
  set_control_mode(ControlMode::Velocity, true); 
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Forced Velocity Control Mode.");
  // --- [END FIX] ---

  enable_torque(true);

  // Get Control Item Info (Original code)
  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  if (goal_position == nullptr) { return return_type::ERROR; }

  const ControlItem * goal_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) { return return_type::ERROR; }

  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) { return return_type::ERROR; }

  const ControlItem * present_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) { return return_type::ERROR; }

  const ControlItem * present_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
  }
  if (present_current == nullptr) { return return_type::ERROR; }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;

  // Add SyncWrite Handlers
  // We still add both in case reset_command tries to write position,
  // but only Velocity handler (index 1) will be used by our modified write()
  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return return_type::ERROR;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return return_type::ERROR;
  }

  // Add SyncRead Handler (Original code)
  uint16_t start_address = std::min(
    control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
  uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
                         control_items_[kPresentVelocityItem]->data_length +
                         control_items_[kPresentCurrentItem]->data_length + 2;
  if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return return_type::ERROR;
  }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    // --- [CUSTOM FIX] ---
    // Only export Velocity command interface as we forced Velocity Mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
    // --- [END FIX] ---
    
    // Original code (commented out):
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }

  return command_interfaces;
}

return_type DynamixelHardware::start()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "start");
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }
  read();
  reset_command();
  write();

  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type DynamixelHardware::stop()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "stop");
  
  // --- [CUSTOM FIX] ---
  // Send zero velocity before stopping and disabling torque
  if (!use_dummy_) {
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Sending zero velocity before stop...");
      for(auto & joint : joints_) {
          joint.command.velocity = 0.0;
      }
      if (write() != return_type::OK) {
          RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), "Failed to send zero velocity during stop.");
      }
      rclcpp::sleep_for(std::chrono::milliseconds(50)); // Wait 50ms for motors to react
      
      // Disable torque (original code didn't do this on stop)
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Disabling torque...");
      enable_torque(false); 
  }
  // --- [END FIX] ---

  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

return_type DynamixelHardware::read()
{
  if (use_dummy_) {
    return return_type::OK;
  }

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);
  std::vector<int32_t> currents(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  const char * log = nullptr;

  // Add retries for SyncRead
  bool sync_read_ok = false;
  int retry_count = 0;
  while (retry_count < 3 && !sync_read_ok) {
    if (dynamixel_workbench_.syncRead(
          kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log)) {
        sync_read_ok = true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "SyncRead failed (Attempt %d): %s", retry_count + 1, log);
        retry_count++;
        rclcpp::sleep_for(std::chrono::milliseconds(10)); // Delay before retry
    }
  }
  if (!sync_read_ok) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "SyncRead failed after retries. Using previous state.");
      return return_type::OK; // Return OK to keep controller running
  }


  // Get data from buffer (add error checking)
  bool get_data_ok = true;
  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentCurrentItem]->address,
        control_items_[kPresentCurrentItem]->data_length, currents.data(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Failed to get Load/Current data: %s", log);
    get_data_ok = false;
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentVelocityItem]->address,
        control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Failed to get Velocity data: %s", log);
    get_data_ok = false;
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentPositionItem]->address,
        control_items_[kPresentPositionItem]->data_length, positions.data(), &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Failed to get Position data: %s", log);
    get_data_ok = false;
  }

  if (get_data_ok) {
    for (uint i = 0; i < ids.size(); i++) {
      joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]);
      joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
      joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]); // Use load/current as effort
    }
  } else {
       RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Failed to extract data via getSyncReadData. Using previous state.");
  }


  return return_type::OK;
}

return_type DynamixelHardware::write()
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      if (!std::isnan(joint.command.velocity)) {
           joint.state.velocity = joint.command.velocity; // Reflect command
      }
    }
    return return_type::OK;
  }

  // --- [CUSTOM FIX] Simplified Write for Velocity Control Only ---
  // This replaces the entire original write() function logic
  
  // Don't write if not started or torque is disabled
  if (status_ != hardware_interface::status::STARTED) { return return_type::OK; }
  if (!torque_enabled_) { return return_type::OK; }

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  const char * log = nullptr;
  bool has_valid_command = false;

  // Prepare velocity commands
  for (uint i = 0; i < ids.size(); i++) {
    double command_velocity = joints_[i].command.velocity;
    if (!std::isnan(command_velocity)) {
        commands[i] = dynamixel_workbench_.convertVelocity2Value(
            ids[i], static_cast<float>(command_velocity));
        has_valid_command = true;
    } else {
        // Send 0 for safety if command is NaN
        commands[i] = 0;
        has_valid_command = true; // Still need to write the zero
    }
  }

  // Execute SyncWrite if needed
  if (has_valid_command) {
      bool sync_write_ok = false;
      int retry_count = 0;
      const int max_retries = 1; // Retry SyncWrite once on failure

      while (retry_count <= max_retries && !sync_write_ok) {
          if (dynamixel_workbench_.syncWrite(
                kGoalVelocityIndex, // Use the velocity SyncWrite handler index (which is 1)
                ids.data(), ids.size(), commands.data(), 1, &log)) {
              sync_write_ok = true;
          } else {
              RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "SyncWrite failed (Attempt %d/%d): %s", retry_count + 1, max_retries + 1, log);
              retry_count++;
              if (retry_count <= max_retries) {
                   rclcpp::sleep_for(std::chrono::milliseconds(5)); // Delay before retry
              }
          }
      }

      if (!sync_write_ok) {
           RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "SyncWrite failed after %d attempts.", max_retries + 1);
           // Return OK to allow controller to continue
           return return_type::OK;
      }
  }

  return return_type::OK;
  // --- [END CUSTOM FIX] ---
}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  const char * log = nullptr;

  // --- [CUSTOM FIX] Add delay and check state ---
  if (enabled == torque_enabled_) { return return_type::OK; } // Already in desired state

  if (enabled) { // Request to enable
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Enabling torque...");
    bool all_enabled = true;
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "   Failed to enable torque for ID %d: %s", joint_ids_[i], log);
        all_enabled = false;
      }
       rclcpp::sleep_for(std::chrono::milliseconds(10)); // Small delay
    }
    if (all_enabled) {
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "   Torque enabled successfully.");
        torque_enabled_ = true;
        reset_command(); // Reset commands after enabling
        return return_type::OK;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "   Failed to enable torque for one or more joints.");
        torque_enabled_ = false; // Mark as not fully enabled
        return return_type::ERROR; // Return error if any failed
    }
  } else { // Request to disable
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Disabling torque...");
    bool all_disabled = true;
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "   Failed to disable torque for ID %d: %s", joint_ids_[i], log);
        all_disabled = false;
      }
       rclcpp::sleep_for(std::chrono::milliseconds(10)); // Small delay
    }
     if(!all_disabled) {
        RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), "   Failed to disable torque for one or more joints.");
     } else {
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "   Torque disabled successfully.");
     }
     torque_enabled_ = false;
     return return_type::OK;
  }
  // --- [END FIX] ---
}

return_type DynamixelHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;
  
  // --- [CUSTOM FIX] Add delay and check state ---
  if (mode == control_mode_ && !force_set) {
      return return_type::OK; // Already in desired mode
  }
  // --- [END FIX] ---

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Setting Velocity Control Mode (force_set=%s)...", force_set ? "true" : "false");
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
      rclcpp::sleep_for(std::chrono::milliseconds(50)); // Delay after torque off
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "   Failed to set Velocity Mode for ID %d: %s", joint_ids_[i], log);
        return return_type::ERROR; // Fail hard
      }
      rclcpp::sleep_for(std::chrono::milliseconds(20)); // Delay between motors
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "   Successfully set Velocity Control Mode.");
    control_mode_ = ControlMode::Velocity;

    if (torque_enabled) {
      rclcpp::sleep_for(std::chrono::milliseconds(50)); // Delay before torque on
      if(enable_torque(true) != return_type::OK) {
           RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "   Failed to re-enable torque after setting Velocity Mode.");
           return return_type::ERROR;
      }
    }
  } else if (
    mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Setting Position Control Mode (force_set=%s)...", force_set ? "true" : "false");
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
      rclcpp::sleep_for(std::chrono::milliseconds(50)); // Delay after torque off
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "   Failed to set Position Mode for ID %d: %s", joint_ids_[i], log);
        return return_type::ERROR; // Fail hard
      }
      rclcpp::sleep_for(std::chrono::milliseconds(20)); // Delay between motors
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "   Successfully set Position Control Mode.");
    control_mode_ = ControlMode::Position;

    if (torque_enabled) {
      rclcpp::sleep_for(std::chrono::milliseconds(50)); // Delay before torque on
      if(enable_torque(true) != return_type::OK) {
           RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "   Failed to re-enable torque after setting Position Mode.");
           return return_type::ERROR;
      }
    }
  } else if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kDynamixelHardware), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type DynamixelHardware::reset_command()
{
  // Reset commands based on current state (if available)
  for (uint i = 0; i < joints_.size(); i++) {
    if (!std::isnan(joints_[i].state.position)) {
        joints_[i].command.position = joints_[i].state.position;
    } else {
        joints_[i].command.position = 0.0; // Default to 0 if state is unknown
    }
    joints_[i].command.velocity = 0.0; // Always reset velocity to 0
    joints_[i].command.effort = 0.0;
  }

  return return_type::OK;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)