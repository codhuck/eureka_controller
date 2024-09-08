#include "eureka_controller/eureka_ackermann_controller.hpp"

namespace eureka_ackermann_controller
{
EurekaAckermannController::EurekaAckermannController()
: eureka_steering_library::EurekaSteeringLibrary()
{
}

void EurekaAckermannController::initialize_implementation_parameter_listener()
{
  ackermann_param_listener_ = std::make_shared<eureka_ackermann_controller::ParamListener>(get_node()); //
}

controller_interface::CallbackReturn EurekaAckermannController::configure_odometry()
{

  ackermann_params_ = ackermann_param_listener_->get_params();

  const double middle_wheels_radius = ackermann_params_.middle_wheels_radius;
  const double middle_wheel_track   = ackermann_params_.middle_wheel_track;

  const double wheelbase = ackermann_params_.half_wheelbase;

  odometry_.set_wheel_params(middle_wheels_radius, wheelbase, middle_wheel_track);
  odometry_.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);

  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  RCLCPP_INFO(get_node()->get_logger(), "ackermann odom configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool EurekaAckermannController::update_odometry(const rclcpp::Duration & period)
{
  if (params_.open_loop)
  {
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
  }
  else
  {
    const double traction_middle_right_wheel_value = state_interfaces_[STATE_TRACTION_MIDDLE_RIGHT_WHEEL].get_value();
    const double traction_middle_left_wheel_value = state_interfaces_[STATE_TRACTION_MIDDLE_LEFT_WHEEL].get_value();
    const double steering_front_right_position = state_interfaces_[STATE_STEER_FRONT_RIGHT_WHEEL].get_value();
    const double steering_front_left_position = state_interfaces_[STATE_STEER_FRONT_LEFT_WHEEL].get_value();

    if (std::isfinite(traction_middle_right_wheel_value) && std::isfinite(traction_middle_left_wheel_value) &&
        std::isfinite(steering_front_right_position)     && std::isfinite(steering_front_left_position))
    {
      if (params_.position_feedback)
      {
        // Estimate linear and angular velocity using join
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(
          traction_middle_right_wheel_value, traction_middle_left_wheel_value, steering_front_right_position,
          steering_front_left_position, period.seconds());
      }
      else
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_velocity(
          traction_middle_right_wheel_value, traction_middle_left_wheel_value, steering_front_right_position,
          steering_front_left_position, period.seconds());
      }
    }
  }
  return true;
}
}  // namespace eureka_ackermann_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  eureka_ackermann_controller::EurekaAckermannController,
  controller_interface::ChainableControllerInterface)
