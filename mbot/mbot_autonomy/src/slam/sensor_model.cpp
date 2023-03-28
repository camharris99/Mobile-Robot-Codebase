#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
    : ray_stride_(1)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t &sample,
                                const mbot_lcm_msgs::lidar_t &scan,
                                const OccupancyGrid &map)
{
    double likelihood = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    // TODO
    for (auto &ray : movingScan)
    {
        Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );
        double l_odds = map.logOdds(f_end.x,f_end.y);
        if(l_odds > 0){
            likelihood = l_odds;
        }else{
            for(int i = -1; i < 2; i++){
                for(int j = -1; j < 2; j++){
                    if(map.logOdds(f_end.x,f_end.y) > 0){
                        likelihood = likelihood + map.logOdds(f_end.x,f_end.y);
                    }
                }
            } 
            likelihood = likelihood/9;
        }
    }
    return likelihood;
}