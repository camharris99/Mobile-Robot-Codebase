#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
using namespace std::chrono; 

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    MovingLaserScan movingScan(scan, previousPose_, pose);

    for(auto& ray : movingScan) {
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your endpoint score ///////////////////////

    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    float value = map.logOdds(end_cell.x, end_cell.y);
    if (value <= (127 - kHitOdds_)) {
        map.setLogOdds(end_cell.x, end_cell.y, value+kHitOdds_);
    }
    
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    //////////////// TODO: Implement your ray score ///////////////////////
    std::vector<Point<int>> cells_touched = bresenham(ray, map);
    float value;
    for (Point<int> point : cells_touched) {
        value = map.logOdds(point.x, point.y);
        if (value >= (-127 + kMissOdds_)) {
            map.setLogOdds(point.x, point.y, value-kMissOdds_);
        }
    }

}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get global positions 
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    std::vector<Point<int>> cells_touched;
    //////////////// TODO: Implement Bresenham's Algorithm ////////////////

    int x0 = start_cell.x;
    int y0 = start_cell.y;
    int x1 = end_cell.x;
    int y1 = end_cell.y;
    int dx = abs(x1-x0);
    int dy = abs(y1-y0);
    int sx = x0<x1 ? 1 : -1;
    int sy = y0<y1 ? 1 : -1;
    int err = dx-dy;
    int x = x0;
    int y = y0;
    Point<int> cur_cell;
    while(x != x1 || y != y1) {
        cur_cell.x = x;
        cur_cell.y = y;
        cells_touched.push_back(cur_cell);
        int e2 = 2*err;

        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }

        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }

    return cells_touched;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    auto end_cell = global_position_to_grid_cell(Point<double>(
        ray.origin.x + ray.range * std::cos(ray.theta),
        ray.origin.y + ray.range * std::sin(ray.theta)
        ), map);
    //////////////// TODO: Implement divide and step ////////////////
    std::vector<Point<int>> cells_touched;
    return cells_touched;
}
