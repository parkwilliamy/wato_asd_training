#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}


void CostmapCore::initializeCostmap() {
    grid_width_ = 1000;
    grid_height_ = 1000;
    grid_ = std::vector<std::vector<int8_t>>(grid_height_, std::vector<int8_t>(grid_width_, -1)); //initialize with -1s, unknown
	resolution_ = 0.1; 
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {

	if (range > (grid_width_/2)*resolution_) range = (grid_width_/2)*resolution_;
	
	//determine the x,y coords relative to the center of the 50x50 grid
	x_grid = (grid_width_)/2+(range*cos(angle) / resolution_);
	y_grid = (grid_height_)/2+(range*sin(angle) / resolution_);


}

void CostmapCore::markObstacle(int x_grid, int y_grid, double range) {
    tracePath(x_grid, y_grid);
	if (range <= grid_width_) grid_[y_grid][x_grid] = 100; //if laser's range is less than or equal to the costmap grid range
    else grid_[y_grid][x_grid] = 0;
}

void CostmapCore::tracePath(int x_grid, int y_grid) {
    int y0 = grid_height_/2;
    int x0 = grid_width_/2;
    int dy = y_grid-y0;
    int dx = x_grid-x0;
    bool diff = abs(dx) >= abs(dy); //1 if horizontal line, 0 if vertical line
    int x_dir = dx < 0 ? -1: 1;
    int y_dir = dy < 0 ? -1: 1;
    int x = x0;
    int y = y0;
    int sx = diff ? 1: 0;
    int sy = !sx; //line will only take steps in either x or y direction
    int i = 0;
    sx *= x_dir;
    sy *= y_dir;

    //convert x and y change to absolute value
    if (x_dir < 0) dx *= -1;
    if (y_dir < 0) dy *= -1;

    if (dx != 0) {

        int p = diff ? 2*dy-dx: 2*dx-dy;

        while (x+sx*i != x_grid && y+sy*i != y_grid) { //fill in cells along path until target square reached

            grid_[y+sy*i][x+sx*i] = 0;
            if (p >= 0) {
                if (diff) {
                    y+=y_dir;
                    p=p-2*dx;
                }
                else {
                    x+=x_dir;
                    p=p-2*dy;
                    
                }
            }
            if (diff) p=p+2*dy;
            else p=p+2*dx;
            ++i;

        }

    }

    else { //vertical line case

        for (int i = 0; i < dy; i++) {
            grid_[y0+sy*i][x0] = 0;
        }

    }


}


void CostmapCore::inflateObstacles() {

	int inflation_radius = 100; 
	int cost_surrounding = 0;
	int distance = 0;

	for (int y = 0; y < grid_height_; ++y) {
		for (int x = 0; x < grid_width_; ++x) {
			if (grid_[y][x] == 100) {

				

				//iterate over all surrounding cells in the inflation radius and apply the cost formula to it
				for (int dy = inflation_radius*-1; dy <= inflation_radius; ++dy) {
					for (int dx = inflation_radius*-1; dx <= inflation_radius; ++dx) {

						if ((y+dy >= 0 && x+dx >= 0) && (y+dy <= grid_height_-1 && x+dx <= grid_width_-1)) {

							if (dy == 0 && dx == 0) continue; //skip origin
							distance = std::max(abs(dx), abs(dy));
							cost_surrounding = 100*(1-(float)distance/inflation_radius);
							if (cost_surrounding > grid_[y+dy][x+dx]) grid_[y+dy][x+dx] = cost_surrounding; //if calculated cost is greater than the cell's present cost

						}
					}
				}

			}
		}
	}

}

int CostmapCore::getWidth() const {
    return grid_width_;
}

int CostmapCore::getHeight() const {
    return grid_height_;
}

double CostmapCore::getResolution() const {
    return resolution_;
}

std::vector<int8_t> CostmapCore::getCostmapFlat() const {
    return costmap_flat_;
}

void CostmapCore::dimReduce() {

	costmap_flat_ = grid_[0];
	for (int i = 1; i < grid_height_; ++i) {
		costmap_flat_.insert(costmap_flat_.end(), grid_[i].begin(), grid_[i].end());
	}

}
}