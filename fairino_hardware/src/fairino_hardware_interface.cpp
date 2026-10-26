// Copyright (c) 2026, Space Robotics Workers S.L.

// Standard library
#include <algorithm>
#include <string>
#include <vector>

// ROS2
#include <angles/angles.h>
#include <hardware_interface/hardware_info.hpp>
#include <pluginlib/class_list_macros.hpp>

// Project
#include "libfairino/include/robot.h"
#include "fairino_version/fairino_version.hpp"

// Class
#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware
{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  info_ = info;

  if (
    hardware_interface::SystemInterface::on_init(info_) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Connect to controller and get version
  controller_ip_ = info_.hardware_parameters["ip_address"];
  controller_port_ = info_.hardware_parameters["ip_port"];
  fairino_version::load_lib(controller_ip_, controller_port_);

  // Check correct number of joints
  if (info_.joints.size() != 6) {
    RCLCPP_FATAL(LOGGER, "Number of joints must be 6, but %zu were found", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check for valid interface configuration
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      // Fairino drivers support only one command type at a time
      RCLCPP_FATAL(
        LOGGER, "Joints must have only 1 command interface, but joint '%s' has %zu",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Get interface names for processing
    std::string command_interface = joint.command_interfaces[0].name;
    std::vector<std::string> state_interfaces;
    for (const auto & iface : joint.state_interfaces) {
      state_interfaces.push_back(iface.name);
    }

    if (command_interface == hardware_interface::HW_IF_POSITION) {
      // No restrictions for position control
      continue;
    } else if (
      command_interface == hardware_interface::HW_IF_VELOCITY ||
      command_interface == hardware_interface::HW_IF_EFFORT) {
      // State interfaces must include position and velocity
      if (
        std::find(
          state_interfaces.begin(), state_interfaces.end(), hardware_interface::HW_IF_POSITION) ==
        joint.state_interfaces.end()) {
        RCLCPP_FATAL(
          LOGGER,
          "Joints set to velocity or effort control must have position and velocity state "
          "interfaces, but joint '%s' is missing position",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      } else if (
        std::find(
          state_interfaces.begin(), state_interfaces.end(), hardware_interface::HW_IF_VELOCITY) ==
        joint.state_interfaces.end()) {
        RCLCPP_FATAL(
          LOGGER,
          "Joints set to velocity or effort control must have position and velocity state "
          "interfaces, but joint '%s' is missing velocity",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      RCLCPP_FATAL(
        LOGGER, "Joints must have position, velocity, or effort control, but joint '%s' has '%s'",
        joint.name.c_str(), command_interface.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
FairinoHardwareInterface::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  // Reset read flags
  read_position_ = false;
  read_velocity_ = false;
  read_acceleration_ = false;
  read_effort_ = false;

  // Export declared state interfaces
  for (size_t idx = 0; idx < info_.joints.size(); ++idx) {
    const auto & joint = info_.joints[idx];

    for (const auto & state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(
          joint.name, state_interface.name, &joint_position_states_[idx]);
        read_position_ = true;
      } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(
          joint.name, state_interface.name, &joint_velocity_states_[idx]);
        read_velocity_ = true;
      } else if (state_interface.name == hardware_interface::HW_IF_ACCELERATION) {
        state_interfaces.emplace_back(
          joint.name, state_interface.name, &joint_acceleration_states_[idx]);
        read_acceleration_ = true;
      } else if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
        state_interfaces.emplace_back(joint.name, state_interface.name, &joint_effort_states_[idx]);
        read_effort_ = true;
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
FairinoHardwareInterface::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

  // Reset write flags
  write_position_ = false;
  write_velocity_ = false;
  write_effort_ = false;

  // Export declared command interfaces
  for (size_t idx = 0; idx < info_.joints.size(); ++idx) {
    const auto & joint = info_.joints[idx];

    for (const auto & command_interface : joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        command_interfaces.emplace_back(
          joint.name, command_interface.name, &joint_position_commands_[idx]);
        write_position_ = true;
      } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
        command_interfaces.emplace_back(
          joint.name, command_interface.name, &joint_velocity_commands_[idx]);
        write_velocity_ = true;
      } else if (command_interface.name == hardware_interface::HW_IF_EFFORT) {
        command_interfaces.emplace_back(
          joint.name, command_interface.name, &joint_effort_commands_[idx]);
        write_effort_ = true;
      }
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(LOGGER, "Activating robot");

  // Initialize states and commands to zero
  for (size_t idx = 0; idx < 6; ++idx) {
    joint_position_states_[idx] = 0.0;
    joint_velocity_states_[idx] = 0.0;
    joint_acceleration_states_[idx] = 0.0;
    joint_effort_states_[idx] = 0.0;

    joint_position_commands_[idx] = 0.0;
    joint_velocity_commands_[idx] = 0.0;
    joint_effort_commands_[idx] = 0.0;
  }

  fr_robot_ = std::make_unique<FRRobot>();  //Create a robot instance

  errno_t returncode = fr_robot_->RPC(_controller_ip.c_str());  //Establish xmlrpc connection
  using namespace std::chrono_literals;
  rclcpp::sleep_for(
    200ms);  //Wait for a while for the controller's rpc connection to be established.
  if (returncode != 0) {
    RCLCPP_INFO(
      LOGGER,
      "Failed to connect to the robotic arm SDK! Please check if the port is already in use.");
    return hardware_interface::CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(LOGGER, "Robotic arm SDK connected successfully!");
  }
  // Perform the first step: read the current state data.
  JointPos jntpos;
  returncode = fr_robot_->GetActualJointPosDegree(0, &jntpos);
  /*
    After obtaining the feedback position, synchronize it to the command position to maintain the current state. If the reading fails, the plug-in cannot be activated.
    Because the wrong feedback position will lead to serious deviations in the initial command position and lead to accidents.
    */
  if (returncode == 0) {
    for (int j = 0; j < 6; j++) {
      joint_position_commands_[j] = jntpos.jPos[j] / 180.0 * M_PI;
    }
    RCLCPP_INFO(
      LOGGER, "Initial instruction location: %f,%f,%f,%f,%f,%f", joint_position_commands_[0],
      joint_position_commands_[1], joint_position_commands_[2], joint_position_commands_[3],
      joint_position_commands_[4], joint_position_commands_[5]);
    RCLCPP_INFO(LOGGER, "机械臂硬件启动成功!");
    return hardware_interface::CallbackReturn::SUCCESS;
  } else {
    RCLCPP_INFO(
      LOGGER,
      "Error reading initial joint angles; hardware cannot start! Please check the communication "
      "data.");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(LOGGER, "Deactivating robot");
  fr_robot_->StopMotion();  // Stop the robot
  fr_robot_->CloseRPC();    // Destroy the instance and disconnect
  fr_robot_.release();
  RCLCPP_INFO(LOGGER, "Robot is deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FairinoHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Read position state if requested
  if (read_position_) {
    JointPos joint_positions;
    error_t ret = fr_robot_->GetActualJointPosDegree(1, &joint_positions);

    if (ret != 0) {
      RCLCPP_ERROR(LOGGER, "Failed to read joint positions with error %d", ret);
      return hardware_interface::return_type::ERROR;
    }

    for (size_t idx = 0; idx < 6; ++idx) {
      joint_position_states_[idx] = angles::to_radians(joint_positions.jPos[idx]);
    }
  }

  // Read velocity state if requested
  if (read_velocity_) {
    float joint_velocities[6];
    error_t ret = fr_robot_->GetActualJointSpeedsDegree(1, joint_velocities);

    if (ret != 0) {
      RCLCPP_ERROR(LOGGER, "Failed to read joint velocities with error %d", ret);
      return hardware_interface::return_type::ERROR;
    }

    for (size_t idx = 0; idx < 6; ++idx) {
      joint_velocity_states_[idx] = angles::to_radians(joint_velocities[idx]);
    }
  }

  // Read acceleration state if requested
  if (read_acceleration_) {
    float joint_accelerations[6];
    error_t ret = fr_robot_->GetActualJointAccDegree(1, joint_accelerations);

    if (ret != 0) {
      RCLCPP_ERROR(LOGGER, "Failed to read joint accelerations with error %d", ret);
      return hardware_interface::return_type::ERROR;
    }

    for (size_t idx = 0; idx < 6; ++idx) {
      joint_acceleration_states_[idx] = angles::to_radians(joint_accelerations[idx]);
    }
  }

  // Read effort state if requested
  if (read_effort_) {
    float joint_efforts[6];
    error_t ret = fr_robot_->GetJointTorques(1, joint_efforts);

    if (ret != 0) {
      RCLCPP_ERROR(LOGGER, "Failed to read joint efforts with error %d", ret);
      return hardware_interface::return_type::ERROR;
    }

    for (size_t idx = 0; idx < 6; ++idx) {
      joint_effort_states_[idx] = joint_efforts[idx];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Write position command if requested
  if (write_position_) {
    // Validate and convert command input
    JointPos joint_positions;
    for (size_t idx = 0; idx < 6; ++idx) {
      double position_command = joint_position_commands_[idx];

      if(!std::isfinite(position_command){
        RCLCPP_ERROR(
          LOGGER, "Joint '%s' received invalid position command %f", info_.joints[idx].name.c_str(),
          position_command);
        return hardware_interface::return_type::ERROR;
      }

      joint_positions.jPos[idx] = angles::to_degrees(position_command);
    }

    // Build and send command
    ExaxisPos external_positions{0, 0, 0, 0};
    error_t ret = fr_robot_->ServoJ(&joint_positions, &external_positions, 0, 0, 0.008, 0, 0);

    if (ret != 0) {
      RCLCPP_ERROR(LOGGER, "Failed to write joint positions with error %d", ret);
      return hardware_interface::return_type::ERROR;
    }
  }

  // Write velocity command if requested
  if (write_velocity_) {
    // Validate and convert command input
    double joint_velocities[6];
    for (size_t idx = 0; idx < 6; ++idx) {
      double velocity_command = joint_velocity_commands_[idx];

      if(!std::isfinite(velocity_command){
        RCLCPP_ERROR(
          LOGGER, "Joint '%s' received invalid velocity command %f", info_.joints[idx].name.c_str(),
          velocity_command);
        return hardware_interface::return_type::ERROR;
      }

      joint_velocities[idx] = angles::to_degrees(velocity_command);
    }

    // Build and send command
    ExaxisPos external_velocities{0, 0, 0, 0};
    error_t ret = fr_robot_->ServoJV(&joint_velocities, &external_velocities, 0, 0, 0.008, 0, 0);

    if (ret != 0) {
      RCLCPP_ERROR(LOGGER, "Failed to write joint velocities with error %d", ret);
      return hardware_interface::return_type::ERROR;
    }
  }

  // Write effort command if requested
  if (write_effort_) {
    // Validate and convert command input
    double joint_efforts[6];
    for (size_t idx = 0; idx < 6; ++idx) {
      double effort_command = joint_effort_commands_[idx];

      if(!std::isfinite(effort_command){
        RCLCPP_ERROR(
          LOGGER, "Joint '%s' received invalid effort command %f", info_.joints[idx].name.c_str(),
          effort_command);
        return hardware_interface::return_type::ERROR;
      }

      joint_efforts[idx] = effort_command;
    }

    // Build and send command
    ExaxisPos external_efforts{0, 0, 0, 0};
    error_t ret = fr_robot_->ServoJV(&joint_efforts, &external_efforts, 0, 0, 0.008, 0, 0);

    if (ret != 0) {
      RCLCPP_ERROR(LOGGER, "Failed to write joint efforts with error %d", ret);
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace fairino_hardware

PLUGINLIB_EXPORT_CLASS(
  fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
