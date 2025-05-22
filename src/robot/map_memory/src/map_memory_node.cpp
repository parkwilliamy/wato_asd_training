#include "map_memory_node.hpp"
#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <algorithm>
using std::placeholders::_1;

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), distance_threshold(1.5) {

	costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, _1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, _1));
	map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
	timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MapMemoryNode::publishMap, this));
	global_map_.header.frame_id = "sim_world";
	global_map_.header.stamp = this->get_clock()->now();

	global_map_.info.width = 30;
    global_map_.info.height = 30;
	global_map_.info.resolution = 1; 
	global_map_.info.origin.position.x = -15.0;
	global_map_.info.origin.position.y = -15.0;
	global_map_.info.origin.position.z = 0.0;
	global_map_.info.origin.orientation.x = 0.0;
	global_map_.info.origin.orientation.y = 0.0;
	global_map_.info.origin.orientation.z = 0.0;
	global_map_.info.origin.orientation.w = 1.0;

	std::vector<int8_t> grid(900, -1);
	global_map_.data = grid;
  
}

// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Store the latest costmap
    latest_costmap_ = *msg;
    costmap_updated_ = true;

}

// Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Compute distance traveled
    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = x;
        last_y = y;
        should_update_map_ = true;
    }

	yaw_ = quaternionToYaw(msg->pose.pose.orientation);

	//RCLCPP_INFO(this->get_logger(), "Last x = %.2f", last_x);
	//RCLCPP_INFO(this->get_logger(), "Last y = %.2f", last_y);

}

// Timer-based map update
void MapMemoryNode::publishMap() {

    if (should_update_map_ && costmap_updated_) {
        integrateCostMap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
    }

}
double MapMemoryNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void MapMemoryNode::integrateCostMap() {
    
    double costmap_origin_x = latest_costmap_.info.origin.position.x;
    double costmap_origin_y = latest_costmap_.info.origin.position.y;

    for (int y = 0; y < latest_costmap_.info.height; ++y) {
        for (int x = 0; x < latest_costmap_.info.width; ++x) {
            // Skip unknown cells
            int8_t value = latest_costmap_.data[y * latest_costmap_.info.width + x];
            if (value == -1) continue;

            // Local costmap coordinates (cell center in meters)
            double local_x = costmap_origin_x + (x + 0.5) * latest_costmap_.info.resolution;
            double local_y = costmap_origin_y + (y + 0.5) * latest_costmap_.info.resolution;

            // Rotate by robot yaw and translate by robot position
            double global_x = std::cos(yaw_) * local_x - std::sin(yaw_) * local_y + last_x;
            double global_y = std::sin(yaw_) * local_x + std::cos(yaw_) * local_y + last_y;
			

            // Convert to global map grid coordinates
            int gx = static_cast<int>((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
            int gy = static_cast<int>((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

            // Bounds check
            if (gx >= 0 && gx < static_cast<int>(global_map_.info.width) &&
                gy >= 0 && gy < static_cast<int>(global_map_.info.height)) {

                int gidx = gy * global_map_.info.width + gx;
                global_map_.data[gidx] = std::max(global_map_.data[gidx], value);
            }
        }
    }
}
	
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}