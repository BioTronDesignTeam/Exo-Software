#include "exo_controller/exo_controller.hpp"
#include <controller_interface/controller_interface_base.hpp>

namespace exo_controller {

CallbackReturn ExoController::on_init() {
    return CallbackReturn::SUCCESS;
}

InterfaceConfiguration ExoController::command_interface_configuration() const {
    InterfaceConfiguration config;
    return config;
}

InterfaceConfiguration ExoController::state_interface_configuration() const {
    InterfaceConfiguration config;
    return config;
}

CallbackReturn ExoController::on_configure(const rclcpp_lifecycle::State & previous_state) {
    return CallbackReturn::SUCCESS;
}

CallbackReturn ExoController::on_activate(const rclcpp_lifecycle::State & previous_state) {
    return CallbackReturn::SUCCESS;
}

CallbackReturn ExoController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    return CallbackReturn::SUCCESS;
}

return_type ExoController::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
    return return_type::OK;
}

CallbackReturn ExoController::on_cleanup(const rclcpp_lifecycle::State & previous_state) {
    return CallbackReturn::SUCCESS;
}

CallbackReturn ExoController::on_error(const rclcpp_lifecycle::State & previous_state) {
    return CallbackReturn::ERROR;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    exo_controller::ExoController,
    controller_interface::ControllerInterface
)