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
    void initializeCostmap(); //initializes costmap array with values of -1, value of -1 is unknown, 0 is free space, and 100 is obstacle
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid); //converts lidar scan data into a costmap array
    void markObstacle(int x_grid, int y_grid, double range); //marks obstacles based on if the range is within the costmap dimensions (in this case, a 5m range)
    void tracePath(int x_grid, int y_grid); //determines the cells that need to be marked as 0 (free space) from the robot to the obstacle point
    void inflateObstacles(); //creates a "gradient" of obstacle cells around a marked cell to indicate more dangerous regions on the map
    void dimReduce(); //converts 2D costmap array into a 1D array so costmap can be sent over the /costmap topic

    //getters for costmap attributes
    int getWidth() const;
    int getHeight() const; 
    double getResolution() const; 
    std::vector<int8_t> getCostmapFlat() const;

  private:
    rclcpp::Logger logger_;
    double resolution_; //defines the real world distance of each grid cell (i.e., 100x100 grid with 0.1 resolution is 10mx10m)
    int grid_width_; 
    int grid_height_;
    std::vector<std::vector<int8_t>> grid_; //2D array used for costmap
    std::vector<int8_t> costmap_flat_;
    bool isObstacle_; //true if range of scan is less than costmap range 
    
};

}  

#endif  