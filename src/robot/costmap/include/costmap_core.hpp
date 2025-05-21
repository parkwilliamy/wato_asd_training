#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    void initializeCostmap();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid, double range);
    void tracePath(int x_grid, int y_grid);
    void inflateObstacles();
    void dimReduce();

    int getWidth() const;
    int getHeight() const;
    double getResolution() const;
    std::vector<int8_t> getCostmapFlat() const;

  private:
    rclcpp::Logger logger_;
    double resolution_;
    int grid_width_;
    int grid_height_;
    std::vector<std::vector<int8_t>> grid_;
    std::vector<int8_t> costmap_flat_;
    
};

}  

#endif  