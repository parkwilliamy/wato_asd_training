#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>

namespace robot {
    class ControlCore {
    public:
        ControlCore(rclcpp::Logger logger) : logger_(logger) {}
    private:
        rclcpp::Logger logger_;
    };
}

class ControlNode : public rclcpp::Node {
public:
    ControlNode();

private:
    // ROS entities
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Robot state
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // Parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    int points_passed;
    double dist_to_goal_ = 10000;

    // Control logic
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target);
    double extractYaw(const geometry_msgs::msg::Quaternion& q);
    void controlLoop();

    // Dummy controller core object (for potential expansion)
    robot::ControlCore control_;
};

#endif // CONTROL_NODE_HPP

