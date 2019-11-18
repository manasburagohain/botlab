#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <cstdlib>
#include <math.h>       /* cos */

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{

    //printf("%f\n",map.logOdds(1,1));
    //map.setLogOdds(1,1,1)

    /*int32_t num_ranges;
    float   ranges[num_ranges];         // [m]
    float   thetas[num_ranges];         // [rad]
    int64_t times[num_ranges];          // [usec]
    float   intensities[num_ranges];    // no units
    */
    //float x = scan.thetas[1];
    //printf("%f\n",x);

    float grid_size = .05; // 5cm

    float x0_m = pose.x; // in meters
    float y0_m = pose.y;

    int x0 = to_grid(x0_m, grid_size); // in grid coordinates
    int y0 = to_grid(y0_m, grid_size);

    // for each laser beam, use breshentham to update map
    num_lasers = scan.num_ranges;
    for (i=1; i<num_lasers; i++){
        float r = scan.ranges[i];
        float theta = scan.thetas[i];
        float x1_m = x0_m + r * cos(theta);
        float y1_m = y0_m + r * sin(theta);
        int x1 = to_grid(x1_m, grid_size);
        int y1 = to_grid(y1_m, grid_size);

        float dx = std::abs(x1-x0);
        float dy = std::abs(y1-y0);
        int sx = x0<x1 ? 1 : -1;
        int sy = y0<y1 ? 1 : -1;
        float err = dx-dy;

        // x,y starting points
        float x = x0;
        float y = y0;
        // change x and y until hit one of end points
        while(x != x1 || y != y1){
            updateOdds(x,y);
            float e2 = 2*err;
            if (e2 >= -dy){
                err -= dy;
                x += sx;
            }
            if (e2 <= dx){
                err += dx
                y += sy
            }
        }

    }
}

int Mapping::to_grid(float x, float grid_size) {
    return floor(x / grid_size);
}

// Next time - updateOdds