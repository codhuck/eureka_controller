#pragma once

#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "diffdrive_arduino/visibility_control.h"
#include "eureka_control_hardware/wheel.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace arckermandrive
{
class Arckermandrive : public hardware_interface::SystemInterface
{

struct Config
{
  std::string steer_front_left_wheel_name = "";
  std::string steer_front_right_wheel_name = "";
  std::string traction_middle_left_wheel_name = "";
  std::string tracrion_middle_right_wheel_name = "";
  std::string steer_rear_left_wheel_name = "";
  std::string steer_rear_right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Arckermandrive);

  ACKERMAN_DRIVE
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ACKERMAN_DRIVE
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ACKERMAN_DRIVE
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ACKERMAN_DRIVE
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMAN_DRIVE
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  ACKERMAN_DRIVE
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMAN_DRIVE
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMAN_DRIVE
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ACKERMAN_DRIVE
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_front_l_;
  Wheel wheel_front_r_;
  Wheel wheel_middle_l_;
  Wheel wheel_middle_r_;
  Wheel wheel_rear_l_;
  Wheel wheel_rear_r_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub2;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub2;

  std::vector<float> vel_filt;
  std::vector<float> ang_wheel;
  std::vector<float> vel_ang_wheel;
  std::vector<float> arr;
  std::vector<float> positions;
  std::vector<float> velocities;
  std::vector<float> efforts;

  void callback(const std_msgs::msg::UInt8MultiArray::SharedPtr arr);
  float convertFloat16ToFloat32(uint8_t byte1, uint8_t byte2);

};

}  