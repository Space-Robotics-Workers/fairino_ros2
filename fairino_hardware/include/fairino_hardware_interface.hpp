// Copyright (c) 2026, Space Robotics Workers S.L.

#ifndef FAIRINO_HARDWARE_INTERFACE_HPP_
#define FAIRINO_HARDWARE_INTERFACE_HPP_

// Standard library
#include <vector>

// ROS2
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

// Project
#include "libfairino2.3.7/robot.h"
#include "visibility_control.h"

namespace fairino_hardware
{
const std::string BUILD_VERSION = "v3.9.7";
const std::string DEFAULT_IP = "192.168.58.2";

class FairinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  /// Initialize all member variables and process the parameters from hardware info
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  /// Setup the communication to the hardware and prepare for activation
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Read-only states are exposed
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
    override;

  /// Write-only commands are exposed
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
    override;

  /// Hardware is enabled and ready to use
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Get the current hardware inputs
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Set the new hardware outputs
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Hardware is disabled
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Handle error state
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Close communication and prepare for shutdown
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  std::string controller_ip_;
  std::unique_ptr<FRRobot> fr_robot_;

  bool read_position_;
  bool read_velocity_;
  bool read_acceleration_;
  bool read_effort_;

  bool write_position_;
  bool write_velocity_;
  bool write_effort_;

  double joint_position_states_[6];
  double joint_velocity_states_[6];
  double joint_acceleration_states_[6];
  double joint_effort_states_[6];

  double joint_position_commands_[6];
  double joint_velocity_commands_[6];
  double joint_effort_commands_[6];

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
};

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE_INTERFACE_HPP_
