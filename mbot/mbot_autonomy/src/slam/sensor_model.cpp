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
    double scanscore;

    for(auto& ray: movingScan){
        Point<double> endpoint(ray.origin.x + ray.range * cos(ray.theta),
        ray.origin.y + ray.range * sin(ray.theta));

        auto rayEnd = global_position_to_grid_position(endpoint,map);
        if(map.logOdds(rayEnd.x,rayEnd.y)>0.0){
            scanscore += 1.0;
        }
    }


    return scanscore;
}