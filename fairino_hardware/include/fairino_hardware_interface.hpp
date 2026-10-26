// Copyright (c) 2026, Space Robotics Workers S.L.

#ifndef FR_HARDWARE_INTERFACE_
#define FR_HARDWARE_INTERFACE_

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
#include "libfairino/include/robot.h"
#include "visibility_control.h"

namespace fairino_hardware
{
const auto LOGGER = rclcpp::get_logger("fairino_hardware_interface");

class FairinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
    override;

  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
    override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string controller_ip_;
  std::string controller_port_;
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
};

}  // namespace fairino_hardware

#endif
