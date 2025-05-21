#include "control_node.hpp"
#include <cmath>
#include <optional>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {

    // Initialize parameters
    lookahead_distance_ = 1;  // Lookahead distance
    goal_tolerance_ = 0.1;      // Distance to consider the goal reached
    linear_speed_ = 1;        // Constant forward speed

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
            current_path_ = msg;
            points_passed = 0;  // Reset on new path
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            robot_odom_ = msg;
        });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { controlLoop(); });

    points_passed = 0;
}

void ControlNode::controlLoop() {
    if (!current_path_ || !robot_odom_) {
        return;
    }

    if (dist_to_goal_ <= goal_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping robot.");
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        RCLCPP_INFO(this->get_logger(), "No valid lookahead point found.");
        return;
    }

    auto cmd_vel = computeVelocity(*lookahead_point);
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    if (!current_path_ || current_path_->poses.empty()) {
        return std::nullopt;
    }

    for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double dx = current_path_->poses[i].pose.position.x - robot_odom_->pose.pose.position.x;
        double dy = current_path_->poses[i].pose.position.y - robot_odom_->pose.pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist >= lookahead_distance_) {
            points_passed = i;
            break;
        }
    }

    return current_path_->poses[points_passed];


}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    geometry_msgs::msg::Twist cmd_vel;

    double yaw = extractYaw(robot_odom_->pose.pose.orientation);
    double dx = target.pose.position.x - robot_odom_->pose.pose.position.x;
    double dy = target.pose.position.y - robot_odom_->pose.pose.position.y;
    double alpha = std::atan2(dy, dx) - yaw;
    dist_to_goal_ = std::sqrt(dx * dx + dy * dy);

    
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 2 * linear_speed_ * std::sin(alpha) / lookahead_distance_;
   

    return cmd_vel;
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

