#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    this->declare_parameter("goal_x", 0.0);
    this->declare_parameter("goal_y", 0.0);
    this->declare_parameter("planner_resolution", 0.05);

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
            goal_received_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }
}

bool PlannerNode::goalReached() {
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5; // Distance threshold for reaching the goal
}

void PlannerNode::planPath() {
    grid = {};

    // Convert robot and goal pose to grid indices
    int x1 = static_cast<int>((robot_pose_.position.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
    int y1 = static_cast<int>((robot_pose_.position.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
    int x2 = static_cast<int>((goal_.point.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
    int y2 = static_cast<int>((goal_.point.y - current_map_.info.origin.position.y) / current_map_.info.resolution);

    PlannerNode::AStarNode end_node = AStarSearch(x1, y1, x2, y2);

    PlannerNode::CellIndex end_idx(x2, y2);
    if (grid.find(end_idx) == grid.end() || grid[end_idx].parent == CellIndex()) { // if there is no valid path
        RCLCPP_ERROR(this->get_logger(), "No path found to goal (%d, %d)", x2, y2); // used for debugging
        return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = "sim_world";
    path.header.stamp = this->get_clock()->now();

    PlannerNode::CellIndex parent_idx = end_node.parent;

    geometry_msgs::msg::PoseStamped temp; // used to store poses to later store in path message
    temp.header.frame_id = "sim_world";
    temp.header.stamp = this->get_clock()->now();
    temp.pose.position.x = current_map_.info.origin.position.x + (x2 + 0.5) * current_map_.info.resolution;
    temp.pose.position.y = current_map_.info.origin.position.y + (y2 + 0.5) * current_map_.info.resolution;
    path.poses.insert(path.poses.begin(), temp); // insert the target node's pose first

    while (parent_idx != CellIndex(x1, y1)) { // while we haven't reached the start node
        temp.pose.position.x = current_map_.info.origin.position.x + (parent_idx.x + 0.5) * current_map_.info.resolution;
        temp.pose.position.y = current_map_.info.origin.position.y + (parent_idx.y + 0.5) * current_map_.info.resolution;
        path.poses.insert(path.poses.begin(), temp);
        parent_idx = grid[parent_idx].parent;
    }

    temp.pose.position.x = current_map_.info.origin.position.x + (x1 + 0.5) * current_map_.info.resolution;
    temp.pose.position.y = current_map_.info.origin.position.y + (y1 + 0.5) * current_map_.info.resolution;
    path.poses.insert(path.poses.begin(), temp);

    path_pub_->publish(path);
}

PlannerNode::AStarNode& PlannerNode::AStarSearch(int x1, int y1, int x2, int y2) {
    PlannerNode::CellIndex start(x1, y1);
    PlannerNode::CellIndex end(x2, y2);
    PlannerNode::CellIndex current = start;
    PriorityQueue open_nodes = {};
    std::vector<PlannerNode::CellIndex> closed_nodes = {};

    for (int y = 0; y < current_map_.info.height; ++y) {
        for (int x = 0; x < current_map_.info.width; ++x) {
            grid[CellIndex(x, y)] = PlannerNode::AStarNode(CellIndex(x, y), 0, 10000, 0, CellIndex());
        }
    }

    grid[start].g_score = 0;
    grid[start].h_score = distance(start, end);
    grid[start].f_score = distance(start, end);
    open_nodes.push(grid[start]);

    while (!open_nodes.empty()) {
        current = open_nodes.top().index;
        open_nodes.pop();
        closed_nodes.push_back(current);
        if (current == end) break;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                PlannerNode::CellIndex neighbor(current.x + dx, current.y + dy);

                if ((current.y + dy >= 0 && current.x + dx >= 0) &&
                    (current.y + dy < current_map_.info.height &&
                     current.x + dx < current_map_.info.width)) {

                    // if an obstacle or node in closed list
                    if (current_map_.data[neighbor.x + current_map_.info.width * neighbor.y] >= 60 ||
                        std::find(closed_nodes.begin(), closed_nodes.end(), neighbor) != closed_nodes.end())
                        continue;

                    double tentative_g = grid[current].g_score + distance(current, neighbor);

                    if (tentative_g < grid[neighbor].g_score ||
                        std::find(open_nodes.begin(), open_nodes.end(), grid[neighbor]) == open_nodes.end()) {
                        grid[neighbor].g_score = tentative_g;
                        grid[neighbor].f_score = tentative_g + distance(neighbor, end);
                        grid[neighbor].parent = current;

                        // If neighbor is not already in open_nodes, push it
                        if (std::find(open_nodes.begin(), open_nodes.end(), grid[neighbor]) == open_nodes.end()) {
                            open_nodes.push(grid[neighbor]);
                        }
                    }
                }
            }
        }
    }

    return grid[end];
}

double PlannerNode::distance(CellIndex p1, CellIndex p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}