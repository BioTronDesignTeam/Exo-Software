#ifndef EXO_CONTROLLER_HPP_
#define EXO_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"

using namespace controller_interface;

namespace exo_controller {
    class ExoController : public ControllerInterface {
        public:
        ExoController() {};

        CallbackReturn on_init() override;

        InterfaceConfiguration command_interface_configuration() const override;

        InterfaceConfiguration state_interface_configuration() const override;

        CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        return_type update(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_error(
            const rclcpp_lifecycle::State & previous_state) override;
    };
}

#endif