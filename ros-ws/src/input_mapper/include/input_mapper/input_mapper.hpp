#ifndef INPUT_MAPPER_NODE_HPP_ 
#define INPUT_MAPPER_NODE_HPP_

#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "exo_msgs/msg/state.hpp"

namespace input_mapper {
    
    class InputMapper : public rclcpp::Node {
        public:
        explicit InputMapper(const rclcpp::NodeOptions &options);
        
        private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub = nullptr;
        rclcpp::Publisher<exo_msgs::msg::State>::SharedPtr exo_state_pub = nullptr;
        
        bool left_knee_control = false;
        bool right_knee_contrlol = false;
    };
    
}

#endif