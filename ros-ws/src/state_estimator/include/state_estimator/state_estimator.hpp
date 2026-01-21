#ifndef STATE_ESTIMATOR_HPP_
#define STATE_ESTIMATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "exo_msgs/msg/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <vector>
#include <string>

namespace state_estimator {

class StateEstimator : public rclcpp::Node {
    public:
    explicit StateEstimator(const rclcpp::NodeOptions &options);

    private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub = nullptr;
    rclcpp::Subscription<exo_msgs::msg::State>::SharedPtr exo_states_sub = nullptr;

    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;

    rclcpp::Time last_time;

    void exo_states_callback(const exo_msgs::msg::State::SharedPtr msg);
};

}

#endif