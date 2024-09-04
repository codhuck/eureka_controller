#include "eureka_control_hardware/hardware.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arckermandrive
{
  Arckermandrive::Arckermandrive():hardware_interface::SystemInterface{
   pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>("can_tx", 10);
   sub= this->create_subscription<std_msgs::msg::UInt8MultiArray>("can_rx", std::bind(&Ackermandrive::callback, this, std::placeholders::_1));
   pub_2 = this->create_publisher<sensor_msgs::msg::JointState>("wheel_commands", 10);
  }



  void Ackermandrive::callback(const std_msgs::msg::UInt8MultiArray arr)
  {
    int index=static_cast<int>(arr->data[0]);
    if (index>10 && index<17)
    {
      float temp=convertFloat16ToFloat32(arr->data[5],arr->data[6])
      if (std::isnan(temp))
      {
        temp=0;
      }
      positions[index-11]=static_cast<float>(temp);
      velocities[index-11]=static_cast<float>(convertFloat16ToFloat32(arr->data[1],arr->data[2]));
      efforts[index-11]=static_cast<float>(convertFloat16ToFloat32(arr->data[3],arr->data[4]));
    }
  }

  float Ackermandrive::convertFloat16ToFloat32(uint8_t byte1, uint8_t byte2) {
        uint16_t h = (static_cast<uint16_t>(byte2) << 8) | byte1;
        uint16_t h_exp = (h & 0x7C00) >> 10; 
        uint16_t h_sig = h & 0x03FF;         
        uint32_t f;

        if (h_exp == 0) {
            if (h_sig == 0) {
                f = (h & 0x8000) << 16; 
            } else {
                h_sig <<= 1;
                while ((h_sig & 0x0400) == 0) {
                    h_sig <<= 1;
                    h_exp--;
                }
                h_exp++;
                h_sig &= ~(0x0400);
                f = ((h & 0x8000) << 16) | ((h_exp + 112) << 23) | (h_sig << 13);
            }
        } else if (h_exp == 0x1F) {
            // INF/NaN
            f = ((h & 0x8000) << 16) | 0x7F800000 | (h_sig << 13);
        } else {
            f = ((h & 0x8000) << 16) | ((h_exp + 112) << 23) | (h_sig << 13);
        }

        float result;
        std::memcpy(&result, &f, sizeof(f));
        return result;
    }

hardware_interface::CallbackReturn Arckermandrive::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.steer_front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.steer_front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.traction_middle_left_wheel_name = info_.hardware_parameters["middle_left_wheel_name"];
  cfg_.tracrion_middle_right_wheel_name = info_.hardware_parameters["middle_right_wheel_name"];
  cfg_.steer_rear_left_wheel_name = info_.hardware_parameters["rear_left_wheel_name"];
  cfg_.steer_rear_right_wheel_name = info_.hardware_parameters["rear_right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  wheel_front_l_.setup(cfg_.steer_front_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_front_r_.setup(cfg_.steer_front_right_wheel_name, cfg_.enc_counts_per_rev);
  wheel_middle_l_.setup(cfg_.traction_middle_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_middle_r_.setup(cfg_.tracrion_middle_right_wheel_name, cfg_.enc_counts_per_rev);
  wheel_rear_l_.setup(cfg_.steer_rear_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_rear_r_.setup(cfg_.steer_rear_right_wheel_name, cfg_.enc_counts_per_rev);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 6)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[2].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[2].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[3].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[3].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[4].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[4].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[5].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[5].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 6)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[3].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[3].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

      if (joint.state_interfaces[4].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[4].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[5].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AckermanDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[5].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Arckermandrive::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_front_l_.name, hardware_interface::HW_IF_POSITION, &wheel_front_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_front_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_l_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_front_l_.name, hardware_interface::HW_IF_POSITION, &wheel_front_l_.from_angle));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_front_l_.name, hardware_interface::HW_IF_POSITION, &wheel_front_l_.vel_angle_from));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_front_r_.name, hardware_interface::HW_IF_POSITION, &wheel_front_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_front_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_r_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_front_r_.name, hardware_interface::HW_IF_POSITION, &wheel_front_r_.from_angle));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_front_r_.name, hardware_interface::HW_IF_POSITION, &wheel_front_r_.vel_angle_from));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_middle_l_.name, hardware_interface::HW_IF_POSITION, &wheel_middle_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_middle_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_middle_l_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_middle_l_.name, hardware_interface::HW_IF_POSITION, &wheel_middle_l_.from_angle));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_middle_l_.name, hardware_interface::HW_IF_POSITION, &wheel_middle_l_.vel_angle_from));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_middle_r_.name, hardware_interface::HW_IF_POSITION, &wheel_middle_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_middle_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_middle_r_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_middle_r_.name, hardware_interface::HW_IF_POSITION, &wheel_middle_r_.from_angle));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_middle_r_.name, hardware_interface::HW_IF_POSITION, &wheel_middle_r_.vel_angle_from));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rear_l_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rear_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_l_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rear_l_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_l_.from_angle));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rear_l_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_l_.vel_angle_from));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rear_r_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rear_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_r_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rear_r_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_r_.from_angle));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_rear_r_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_r_.vel_angle_from));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Arckermandrive::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_front_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_l_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_front_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_r_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_middle_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_middle_l_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_middle_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_middle_r_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_rear_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_l_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_rear_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_r_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_front_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_l_.vel_angle_to));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_front_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_r_.vel_angle_to));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_middle_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_middle_l_.vel_angle_to));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_middle_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_middle_r_.vel_angle_to));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_rear_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_l_.vel_angle_to));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_rear_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_r_.vel_angle_to));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_front_l_.name, hardware_interface::HW_IF_POSITION, &wheel_front_l_.angle_to));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_front_r_.name, hardware_interface::HW_IF_POSITION, &wheel_front_r_.angle_to));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_middle_l_.name, hardware_interface::HW_IF_POSITION, &wheel_middle_l_.angle_to));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_middle_r_.name, hardware_interface::HW_IF_POSITION, &wheel_middle_r_.angle_to));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_rear_l_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_l_.angle_to));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_rear_r_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_r_.angle_to));

  return command_interfaces;
}

hardware_interface::CallbackReturn Arckermandrive::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AckermanDriveHardware"), "Configuring ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("AckermanDriveHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arckermandrive::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AckermanDriveHardware"), "Cleaning up ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("AckermanDriveHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn Arckermandrive::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AckermanDriveHardware"), "Activating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("AckermanDriveHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arckermandrive::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AckermanDriveHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("AckermanDriveHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Arckermandrive::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  wheel_front_l_.pos=positions[0];
  wheel_front_l_.vel=velocities[0];
  wheel_front_r_.pos=positions[1];
  wheel_front_r_.vel=velocities[1];
  wheel_middle_l_.pos=positions[2];
  wheel_middle_l_.vel=velocities[2];
  wheel_middle_r_.pos=positions[3];
  wheel_middle_r_.vel=velocities[3];
  wheel_rear_l_.pos=positions[4];
  wheel_rear_l_.vel=velocities[4];
  wheel_rear_r_.pos=positions[5];
  wheel_rear_r_.vel=velocities[5];
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type arckermandrive::Arckermandrive::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std_msgs::msg::UInt8MultiArray msg;
  vel_filt={wheel_front_l_.cmd, wheel_front_r_.cmd, wheel_middle_l_.cmd, wheel_middle_r_.cmd, wheel_rear_l_.cmd, wheel_rear_r_.cmd};
  ang_wheel={wheel_front_l_.angle_to, wheel_front_r_.angle_to, wheel_middle_l_.angle_to, wheel_middle_r_.angle_to, wheel_rear_l_.angle_to, wheel_rear_r_.angle_to};
  vel_ang_wheel={wheel_front_l_.vel_angle_to, wheel_front_r_.vel_angle_to, wheel_middle_l_.vel_angle_to, wheel_middle_r_.vel_angle_to, wheel_rear_l_.vel_angle_to, wheel_rear_r_.vel_angle_to};
  for (size_t i=0;i<6;++i)
  {
   std::vector<float> arr={vel_filt[i], ang_wheel[i], vel_ang_wheel[i]};
   std::vector<uint8_t> data=arr
   std::vector<uint8_t> data(1 + arr.size() * sizeof(float));
   data[0] = static_cast<uint8_t>(i + 11);
   std::memcpy(data.data() + 1, arr.data(), arr.size() * sizeof(float));
   msg.data = data;
   pub->publish(msg); 
  }
  return hardware_interface::return_type::OK;
}

}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arckermandrive::Arckermandrive, hardware_interface::SystemInterface)