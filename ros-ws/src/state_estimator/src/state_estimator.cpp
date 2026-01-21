#include "state_estimator/state_estimator.hpp"

namespace {
    constexpr auto DEFAULT_EXO_STATES_TOPIC = "/exo_states";
    constexpr auto DEFAULT_JOINT_STATES_TOPIC = "/joint_states"; 
    //TODO: make a simulation node
}

namespace state_estimator {

StateEstimator::StateEstimator(const rclcpp::NodeOptions &options) : Node("state_estimator", options) {
    exo_states_sub = create_subscription<exo_msgs::msg::State>(
        DEFAULT_EXO_STATES_TOPIC,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&StateEstimator::exo_states_callback, this, std::placeholders::_1)
    );
    joint_state_pub = create_publisher<sensor_msgs::msg::JointState>(
        DEFAULT_JOINT_STATES_TOPIC,
        rclcpp::SystemDefaultsQoS()
    );

    joint_names = {
        "left_hip_joint",
        "left_knee_joint",
        "right_hip_joint",
        "right_knee_joint"
    };
    joint_positions.resize(4, 0);
    joint_velocities.resize(4, 0);
    joint_efforts.resize(4, 0);
    last_time = now();
}

// TODO: implement kalman filter
void StateEstimator::exo_states_callback(const exo_msgs::msg::State::SharedPtr msg) {
    const rclcpp::Time current_time = now();
    const double dt = (current_time - last_time).seconds();
    last_time = current_time;

    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = now();
    joint_state_msg.name = joint_names;

    const double speed_factor = 1.0;

    joint_positions[0] += msg->left_hip_vel * dt * speed_factor;
    joint_positions[1] += msg->left_knee_vel * dt * speed_factor;
    joint_positions[2] += msg->right_hip_vel * dt * speed_factor;
    joint_positions[3] += msg->right_knee_vel * dt * speed_factor;

    joint_velocities[0] = msg->left_hip_vel;
    joint_velocities[1] = msg->left_knee_vel;
    joint_velocities[2] = msg->right_hip_vel;
    joint_velocities[3] = msg->right_knee_vel;

    joint_efforts[0] = msg->left_hip_torque;
    joint_efforts[1] = msg->left_knee_torque;
    joint_efforts[2] = msg->right_hip_torque;
    joint_efforts[3] = msg->right_knee_torque;

    joint_state_msg.position = joint_positions;
    joint_state_msg.velocity = joint_velocities;
    joint_state_msg.effort = joint_efforts;

    joint_state_pub->publish(joint_state_msg);
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(state_estimator::StateEstimator);