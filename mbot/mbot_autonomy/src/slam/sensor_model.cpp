#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(5)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, const mbot_lcm_msgs::lidar_t& scan, const OccupancyGrid& map)
{
    double scanScore = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);


    for(auto& ray : movingScan){
        double rayScore = scoreRay(ray,map);
        scanScore += rayScore;
    }
    return scanScore;
}


double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map){

    Point<float> f_end = global_position_to_grid_position(Point<float>(ray.origin.x + ray.range * std::cos(ray.theta), ray.origin.y + ray.range * std::sin(ray.theta)), map);
    Point<float> f_end_extended = global_position_to_grid_position(Point<float>(ray.origin.x + 2.0*ray.range * std::cos(ray.theta), ray.origin.y + 2.0*ray.range * std::sin(ray.theta)), map);

    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    Point<int> end_cell_extended;

    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    end_cell_extended.x = static_cast<int>(f_end_extended.x);
    end_cell_extended.y = static_cast<int>(f_end_extended.y);

    double odds_current_cell = map.logOdds((end_cell.x), (end_cell.y));

    if (odds_current_cell <= 0.0) {
        odds_current_cell = 0.0;
        int odds_prev_cell = getCellodds(end_cell.x, end_cell.y, start_cell.x, start_cell.y, map);
        int odds_next_cell = getCellodds(end_cell.x, end_cell.y, end_cell_extended.x, end_cell_extended.y, map);

        if (odds_prev_cell > 0.0) {
            odds_current_cell += 0.5 * odds_prev_cell;
        }
        else if (odds_prev_cell > 0.0){
            odds_current_cell += 0.5 * odds_prev_cell;
        }
    }

    // std::cout << "f_end: " << f_end.x << ", " << f_end.y << "... f_end_extended: "  << f_end_extended.x << ", " << f_end_extended.y << std::endl;

    return odds_current_cell;
}

int SensorModel::getCellodds(int x0, int y0, int x1, int y1, const OccupancyGrid& map)
{
    int dx = abs(x1-x0);
    int dy = abs(y1-y0);
    int sx = x0<x1 ? 1 : -1;
    int sy = y0<y1 ? 1 : -1;
    int err = dx-dy;
    int x = x0;
    int y = y0;
    double factor;

    int e2 = 2*err;

    float odds = 0.0;
    for (int i = 0; i < 2; i++) {
        if(e2 >= -dy){
            err -= dy;
            x += sx;
        }
        if(e2 <= dx){
            err += dx;
            y += sy;
        }
        if (i == 0) {
            factor = 1;
        } else {
            factor = 0.5;
        }
        odds += factor*map.logOdds(x,y);
    }
	
    return odds;
}