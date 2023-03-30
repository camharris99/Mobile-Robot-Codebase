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
    double scanLH = 0.0;
    for (auto &ray : movingScan)
    {
        scanLH = 0.0;
        Point<float> f_end = global_position_to_grid_position(
            Point<float>(
                ray.origin.x + ray.range * std::cos(ray.theta),
                ray.origin.y + ray.range * std::sin(ray.theta)),
            map);

        // Cells
        Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
        Point<int> end_cell;
        end_cell.x = static_cast<int>(f_end.x);
        end_cell.y = static_cast<int>(f_end.y);

        double l_odds = map.logOdds(end_cell.x, end_cell.y);
        if (l_odds > 0)
        {
            scanLH = l_odds;
        }
        else
        {
            for (int i = -1; i < 2; i++)
            {
                for (int j = -1; j < 2; j++)
                {
                    l_odds = map.logOdds(end_cell.x+i, end_cell.y+j);
                    if (l_odds > 0)
                    {
                        scanLH += l_odds/9;
                    }
                }
            }
        }
    }
    likelihood += scanLH;
    return likelihood;
}