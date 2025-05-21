#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "planner_core.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <vector>
#include <unordered_map>
#include <algorithm>

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

    std::mutex state_mutex_;

    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

     // Helper structs
    struct CellIndex
    {
      int x;
      int y;
    
      CellIndex(int xx, int yy) : x(xx), y(yy) {}
      CellIndex() : x(0), y(0) {}
    
      bool operator==(const CellIndex &other) const
      {
        return (x == other.x && y == other.y);
      }
    
      bool operator!=(const CellIndex &other) const
      {
        return (x != other.x || y != other.y);
      }
    };

    // Hash function for CellIndex so it can be used in std::unordered_map
  struct CellIndexHash
  {
    std::size_t operator()(const CellIndex &idx) const
    {
      // A simple hash combining x and y
      return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
  };
  
  // Structure representing a node in the A* open set
  struct AStarNode
  {
    CellIndex index;
    double f_score;  // f = g + h
    double g_score;
    double h_score;
    CellIndex parent;
  
    AStarNode(): index(CellIndex()), f_score(0), g_score(10000), h_score(0), parent(CellIndex()) {}
    AStarNode(CellIndex idx, double f, double g, double h, CellIndex p) : index(idx), f_score(f), g_score(g), h_score(h), parent(p) {}
    bool operator==(const AStarNode& other) const {
    return index == other.index;
    }
  };
  
  // Comparator for the priority queue (min-heap by f_score)
  struct CompareF
  {
    bool operator()(const AStarNode &a, const AStarNode &b)
    {
      // We want the node with the smallest f_score on top
      return a.f_score > b.f_score;
    }
  };

   

    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    bool goalReached();
    void planPath();
    AStarNode& AStarSearch(int x1, int y1, int x2, int y2);
    double distance(CellIndex p1, CellIndex p2);

private:
    

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data Storage
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;
    bool goal_received_ = false;

    std::unordered_map<CellIndex, AStarNode, CellIndexHash> grid;

   
};


// Priority queue wrapper
class PriorityQueue {
public:
    void push(const PlannerNode::AStarNode& node);
    void pop();
    PlannerNode::AStarNode& top() const;
    bool empty() const;
    size_t size() const;
    void heapUp(int idx);
    void heapDown(int idx);
    PlannerNode::CompareF comp;
    
    std::vector<PlannerNode::AStarNode>::iterator begin() {
        return pq_.begin();
    }

    std::vector<PlannerNode::AStarNode>::iterator end() {
        return pq_.end();
    }

private:
    
    std::vector<PlannerNode::AStarNode> pq_;
    

};

#endif  