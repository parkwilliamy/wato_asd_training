#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishMap();
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);
    void integrateCostMap(); //integrates costmap into global map based on robot's global pose

  private:
    robot::MapMemoryCore map_memory_;
    nav_msgs::msg::OccupancyGrid latest_costmap_; //costmap message that is received from the costmap node
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    nav_msgs::msg::OccupancyGrid global_map_;
    const double distance_threshold; //distance robot needs to travel before map update is triggered, this is done to avoid redundancies in map updates
    double last_x, last_y; //used to compute distance robot has travelled between timer updates
    bool costmap_updated_ = false;
    bool should_update_map_ = false;
    double yaw_;

};

#endif 