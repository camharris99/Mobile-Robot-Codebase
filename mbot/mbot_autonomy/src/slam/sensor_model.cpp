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

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    double likelihood = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    // TODO
    // double scanscore;

    // for(auto& ray: movingScan){
    //     Point<double> endpoint(ray.origin.x + ray.range * cos(ray.theta),
    //     ray.origin.y + ray.range * sin(ray.theta));

    //     auto rayEnd = global_position_to_grid_position(endpoint,map);
    //     if(map.logOdds(rayEnd.x,rayEnd.y)>0.0){
    //         scanscore += 1.0;
    //     }
    // }

    for(auto& ray : movingScan){
        double rayScore = scoreRay(ray, map);
        likelihood += rayScore;
    }

    return likelihood;
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map){
    Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
    Point<int> rayEnd;

    rayEnd.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    rayEnd.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

    double score = map.logOdds(rayEnd.x, rayEnd.y);
    if(score <= 0.0){
        score = 0.0;
        int dx = std::abs(rayEnd.x - rayStart.x);
        int dy = std::abs(rayEnd.y - rayStart.y);
        int sx = rayEnd.x<rayStart.x ? 1 : -1;
        int sy = rayEnd.y<rayStart.y ? 1 : -1;
        int err = dx - dy;
        int x_after = rayEnd.x;
        int x_before = rayEnd.x;
        int y_after = rayEnd.y;
        int y_before = rayEnd.y;

        int e2 = 2*err;
        if(e2 >= -dy){
            err -= dy;
            x_before = rayEnd.x + sx;
            x_after = rayEnd.x - sx;
        }
        if(e2 <= dx){
            err += dx;
            y_before = rayEnd.y + sy;
            y_after = rayEnd.y - sy;
        }
        double odd_before = map.logOdds(x_before,y_before); 
        double odd_after = 0.0;
        if(map.isCellInGrid(x_after, y_after))    
            odd_after = map.logOdds(x_after, y_after);
        else
            odd_after = 0.0;


        if(odd_before > 0.0)
            score = 0.75 * odd_before;
        else if(odd_after > 0.0)
            score = 0.75 * odd_after;
    }
    return score;
}