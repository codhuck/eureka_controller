#ifndef EUREKA_ACKERMANN_CONTROLLER__EUREKA_ACKERMANN_CONTROLLER_HPP_
#define EUREKA_ACKERMANN_CONTROLLER__EUREKA_ACKERMANN_CONTROLLER_HPP_

#include <memory>

#include "eureka_ackermann_controller/visibility_control.h"
#include "eureka_ackermann_controller_parameters.hpp"
#include "eureka_steering_library/eureka_steering_library.hpp"

namespace eureka_ackermann_controller
{
// name constants for state interfaces
static constexpr size_t STATE_STEER_FRONT_RIGHT_WHEEL = 0;
static constexpr size_t STATE_STEER_FRONT_LEFT_WHEEL = 1;
static constexpr size_t STATE_TRACTION_MIDDLE_RIGHT_WHEEL = 2;
static constexpr size_t STATE_TRACTION_MIDDLE_LEFT_WHEEL = 3;
static constexpr size_t STATE_STEER_REAR_RIGHT_WHEEL = 4;
static constexpr size_t STATE_STEER_REAR_LEFT_WHEEL = 5;

// name constants for command interfaces
static constexpr size_t CMD_STEER_FRONT_RIGHT_WHEEL = 0;
static constexpr size_t CMD_STEER_FRONT_LEFT_WHEEL = 1;
static constexpr size_t CMD_TRACTION_MIDDLE_RIGHT_WHEEL = 2;
static constexpr size_t CMD_TRACTION_MIDDLE_LEFT_WHEEL = 3;
static constexpr size_t CMD_STEER_REAR_RIGHT_WHEEL = 4;
static constexpr size_t CMD_STEER_REAR_LEFT_WHEEL = 5;

// количество интерфейсов, которые будет требовать софтварный контроллер, от хардварного для составления
// статистики (одометрии в данном случае)
static constexpr size_t NR_STATE_ITFS = 6;
// количество интерфейсов которые нужные хардварному контроллеру, чтобы передать от текущего (софтварного)
// контроллера управляющие воздействия на двигатели
static constexpr size_t NR_CMD_ITFS = 6;
// количество интерфейсов для работы логики контроллера ( угловая скорость и линейная )
static constexpr size_t NR_REF_ITFS = 2;

class EurekaAckermannController : public eureka_steering_library::EurekaSteeringLibrary
{
public:
  EurekaAckermannController();

  EUREKA_ACKERMANN_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn
  configure_odometry() override;

  EUREKA_ACKERMANN_CONTROLLER__VISIBILITY_PUBLIC void
  initialize_implementation_parameter_listener() override;

  EUREKA_ACKERMANN_CONTROLLER__VISIBILITY_PUBLIC bool update_odometry(
    const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<eureka_ackermann_controller::ParamListener> ackermann_param_listener_;
  eureka_ackermann_controller::Params ackermann_params_;
};
}  // namespace eureka_ackermann_controller

#endif  // EUREKA_ACKERMANN_CONTROLLER__EUREKA_ACKERMANN_CONTROLLER_HPP_
