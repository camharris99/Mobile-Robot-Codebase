#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(1)
{
}


// double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
//                                const mbot_lcm_msgs::lidar_t& scan, 
//                                const OccupancyGrid& map)
// {
//     double scanScore = 0.0;
//     MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
//     // TODO

//     for (auto& ray : movingScan) {
//         Point<double> endPoint(ray.origin.x + ray.range * std::cos(ray.theta),
//                                ray.origin.y + ray.range * std::sin(ray.theta));
//         auto rayEnd = global_position_to_grid_position(endPoint, map);
//         auto nextCell = rayEnd;
//         nextCell.x += 1;
//         nextCell.y += 1;

//         auto prevCell = rayEnd;
//         prevCell.x -= 1;
//         prevCell.y -= 1;

//         if (map.logOdds(rayEnd.x, rayEnd.y) > 0.0) {
//             scanScore += map.logOdds(rayEnd.x, rayEnd.y);
//         } else if (map.logOdds(nextCell.x, nextCell.y) > 0.0) {
//             scanScore += 0.5*map.logOdds(nextCell.x, nextCell.y);
//         } else if (map.logOdds(prevCell.x, prevCell.y) > 0.0) {
//             scanScore += 0.5*map.logOdds(prevCell.x, prevCell.y);
//         }
//     }
    
//     return scanScore;
// }

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
    Point<float> ray_origin_grid = global_position_to_grid_position(ray.origin, map);
    Point<int> ray_end;
    Point<int> ray_extended;

    ray_end.x = ray.range * std::cos(ray.theta) + ray_origin_grid.x;
    ray_end.y = ray.range * std::sin(ray.theta) + ray_origin_grid.y;

    ray_extended.x = 1.5 * ray.range * std::cos(ray.theta) + ray_origin_grid.x;
    ray_extended.y = 1.5 * ray.range * std::sin(ray.theta) + ray_origin_grid.y;

    double odds_current_cell = map.logOdds(ray_end.x, ray_end.y);

    if (odds_current_cell <= 0.0) {
        odds_current_cell = 0.0;
        int odds_prev_cell = getCellodds(ray_end.x, ray_end.y, ray_origin_grid.x, ray_origin_grid.y, map);
        int odds_next_cell = getCellodds(ray_end.x, ray_end.y, ray_extended.x, ray_extended.y, map);

        if (odds_prev_cell > 0.0) {
            odds_current_cell += 0.5 * odds_prev_cell;
        }
        else if (odds_prev_cell > 0.0){
            odds_current_cell += 0.5 * odds_prev_cell;
        }
    }
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

    int e2 = 2*err;
    if(e2 >= -dy){
        err -= dy;
        x += sx;
    }
    if(e2 <= dx){
        err += dx;
        y += sy;
    }
	
    return map.logOdds(x,y);
}