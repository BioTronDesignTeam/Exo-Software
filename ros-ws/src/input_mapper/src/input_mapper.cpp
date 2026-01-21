#include "input_mapper/input_mapper.hpp"

namespace {
    constexpr auto DEFAULT_JOYSTICK_TOPIC = "/joy";
    constexpr auto DEFAULT_EXO_STATE_TOPIC = "/exo_states";
}

namespace input_mapper {
    
InputMapper::InputMapper(const rclcpp::NodeOptions &options) : Node("input_mapper", options) {
    exo_state_pub = create_publisher<exo_msgs::msg::State>(
        DEFAULT_EXO_STATE_TOPIC,
        rclcpp::SystemDefaultsQoS()
    );
    joystick_sub = create_subscription<sensor_msgs::msg::Joy>(
        DEFAULT_JOYSTICK_TOPIC,
        rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::SharedPtr msg){
            exo_msgs::msg::State state_msg;
            left_knee_control = msg->buttons[4]; //L1 
            right_knee_contrlol = msg->buttons[5]; //R1
                
            double left_vel = msg->axes[1]; // left stick y
            double right_vel = msg->axes[5]; // right stick y

            if (std::abs(left_vel) <= 0.1) {
                left_vel = 0;
            }
            if (std::abs(right_vel) <= 0.1) {
                right_vel = 0;
            }
                
            if (left_knee_control) {
                state_msg.left_knee_vel = left_vel;
            } else {
                state_msg.left_hip_vel = left_vel;
            }
                
            if (right_knee_contrlol) {
                state_msg.right_knee_vel= right_vel;
            } else {
                state_msg.right_hip_vel = right_vel;
            }
             
            exo_state_pub->publish(state_msg);
        });
}
        
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(input_mapper::InputMapper);