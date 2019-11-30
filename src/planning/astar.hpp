#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <math.h>

#define XGRID 1000
#define YGRID 1000
class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};

struct Node
{
    int y;
    int x;
    int parentX;
    int parentY;
    float gCost;
    float hCost;
    float oCost; 
    float fCost;
};

inline bool operator < (const Node& lhs, const Node& rhs)
{//We need to overload "<" to put our struct into a set
    return lhs.fCost < rhs.fCost;
}

static bool isValid(int x, int y, const ObstacleDistanceGrid& distances, double minDist) { //If our Node is an obstacle it is not valid
    /*
    if (distances[x][y] < minDist) {
        if (x < 0 || y < 0 || x >= (XGRID) || y >= (YGRID)) {
            return false;
        }
        return true;
    } 
    return false;
    */
   return false;
}

static bool isDestination(int x, int y, Node dest) {
    if (x == dest.x && y == dest.y) {
        return true;
    }
    return false;
}

static double calculateH(int x, int y, Node dest) {
    double H = (sqrt((x - dest.x)*(x - dest.x)
        + (y - dest.y)*(y - dest.y)));
    return H;
}

static double calculateO(int x, int y, const ObstacleDistanceGrid& distances, const SearchParams& params) 
{
    /*
    int dist = distances[x][y];
    if (dist >= params.maxDistanceWithCost)    return 0;
    else
    {
        ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
        ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
        return    pow(params.maxDistanceWithCost - dist, params.distanceCostExponent);
    }
    */
    return 0.0;
}

/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

#endif // PLANNING_ASTAR_HPP
