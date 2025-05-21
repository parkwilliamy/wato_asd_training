#include <chrono>
#include <memory>
#include <cmath>
#include "costmap_node.hpp"
using std::placeholders::_1;
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  	// Initialize the constructs and their parameters
	costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
	laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, _1));
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishCostmap() {
	auto msg = nav_msgs::msg::OccupancyGrid();

	msg.header.frame_id = "sim_world";
	msg.header.stamp = this->get_clock()->now();

	msg.info.resolution = costmap_.getResolution();
	msg.info.width = costmap_.getWidth();
	msg.info.height = costmap_.getHeight();
	msg.info.origin.position.x = -costmap_.getWidth() / 2.0 * costmap_.getResolution();
	msg.info.origin.position.y = -costmap_.getHeight() / 2.0 * costmap_.getResolution();
	msg.info.origin.position.z = 0.0;
	msg.info.origin.orientation.x = 0.0;
	msg.info.origin.orientation.y = 0.0;
	msg.info.origin.orientation.z = 0.0;
	msg.info.origin.orientation.w = 1.0;

	msg.data = costmap_.getCostmapFlat();

	//RCLCPP_INFO(this->get_logger(), "Publishing costmap with %zu cells", msg.data.size());

	costmap_pub_->publish(msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    costmap_.initializeCostmap();
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            costmap_.convertToGrid(range, angle, x_grid, y_grid);
            costmap_.markObstacle(x_grid, y_grid, range);
        }
    }
 
    // Step 3: Inflate obstacles
    costmap_.inflateObstacles();

	// Step 4: Flatten costmap into 1D array
	costmap_.dimReduce();
 
    // Step 5: Publish costmap
    publishCostmap();

	
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}