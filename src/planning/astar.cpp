#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>
#include <array>
#include <stack>
#include <cfloat>
using namespace std;

robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{

    ////////////////// TODO: Implement your A* search here //////////////////////////
    Node startnode, dest;
    
	robot_path_t path;
    path.utime = start.utime;
    //path.path.push_back(start);    

    //convert to grid cell
    dest.x = (int)goal.x * distances.cellsPerMeter() + distances.originInGlobalFrame().x;
    dest.y = (int)goal.y * distances.cellsPerMeter() + distances.originInGlobalFrame().y;
    startnode.x = (int)start.x * distances.cellsPerMeter() + distances.originInGlobalFrame().x;
    startnode.y = (int)start.y * distances.cellsPerMeter() + distances.originInGlobalFrame().y;
    if (!isValid(dest.x, dest.y, distances, params.minDistanceToObstacle))
    {
        printf("Destination is cannot be reached");
        return path;
    }    

    if(isDestination(startnode.x, startnode.y, dest))
    {
        printf("Already at Destination");
		path.path_length = path.path.size();
        return path;
    }

    bool closedList[distances.widthInCells()][distances.heightInCells()];
    vector< vector<Node>> allMap;
    for (int x = 0; x < (distances.widthInCells()); x++) 
    {
			for (int y = 0; y < (distances.heightInCells()); y++) {
				allMap[x][y].fCost = FLT_MAX;
				allMap[x][y].gCost = FLT_MAX;
				allMap[x][y].hCost = FLT_MAX;
				allMap[x][y].parentX = -1;
				allMap[x][y].parentY = -1;
				allMap[x][y].x = x;
				allMap[x][y].y = y;

				closedList[x][y] = false;
			}
		}
    
    int x = startnode.x;
	int y = startnode.y;
	allMap[x][y].fCost = 0.0;
	allMap[x][y].gCost = 0.0;
	allMap[x][y].hCost = 0.0;
	allMap[x][y].parentX = x;
	allMap[x][y].parentY = y;
    
    vector<Node> openList;	
	openList.emplace_back(allMap[x][y]);
	bool destinationFound = false;

    while (!openList.empty()&&openList.size()<(distances.widthInCells())*(distances.heightInCells())) 
    {
			Node node;
			do 
            {
				//This do-while loop could be replaced with extracting the first
				//element from a set, but you'd have to make the openList a set.
				//To be completely honest, I don't remember the reason why I do
				//it with a vector, but for now it's still an option, although
				//not as good as a set performance wise.
				float temp_f = FLT_MAX;
                float temp_h = FLT_MAX;
				vector<Node>::iterator itNode;
				for (vector<Node>::iterator it = openList.begin();
					it != openList.end(); it = next(it)) 
                {
					Node n = *it;
					if (n.fCost < temp_f) 
                    {
						temp_f = n.fCost;
                        temp_h = n.hCost;
						itNode = it;
					}
                    else if (n.fCost == temp_f) 
                    {
                        if (n.hCost < temp_h) {
                            temp_f = n.fCost;
                            temp_h = n.hCost;
						    itNode = it;
                        }
					}
				}
				node = *itNode;
				openList.erase(itNode);
			} while (isValid(node.x, node.y, distances, params.minDistanceToObstacle) == false);

			x = node.x;
			y = node.y;
			closedList[x][y] = true;

			//For each neighbour starting from North-West to South-East
			for (int newX = -1; newX <= 1; newX++) 
            {
				for (int newY = -1; newY <= 1; newY++) 
                {
					double gNew, hNew, oNew, fNew;
					if (isValid(x + newX, y + newY,distances, params.minDistanceToObstacle)) //add isValid()
                    {
						if (isDestination(x + newX, y + newY, dest)) //add isGoal
						{
							//Destination found - make path
							allMap[x + newX][y + newY].parentX = x;
							allMap[x + newX][y + newY].parentY = y;
							destinationFound = true;
							//path = makePath(allMap, dest, distances);

							/////// make a path
							cout << "Found a Path" << endl;
							int x = dest.x;
							int y = dest.y;
							stack<pose_xyt_t> path;
							robot_path_t usablePath;
							usablePath.utime = start.utime;

							while (!(allMap[x][y].parentX==x && allMap[x][y].parentY==y) && allMap[x][y].x != -1 && allMap[x][y].y != -1)
							{
								pose_xyt_t tempnode;
								tempnode.x = (allMap[x][y].x - distances.originInGlobalFrame().x) * distances.metersPerCell();
								tempnode.y = (allMap[x][y].y - distances.originInGlobalFrame().y) * distances.metersPerCell();
								path.push(tempnode);
								int tempX = allMap[x][y].parentX;
								int tempY = allMap[x][y].parentY;
								x = tempX;
								y = tempY;
							}
							pose_xyt_t node;
							node.x = (allMap[x][y].x - distances.originInGlobalFrame().x) * distances.metersPerCell();
							node.y = (allMap[x][y].y - distances.originInGlobalFrame().y) * distances.metersPerCell();
							path.push(node);

							while (!path.empty())
							{
								pose_xyt_t top = path.top();
								path.pop();
								usablePath.path.emplace_back(top);
							}
							usablePath.path_length = usablePath.path.size();
							return usablePath;


						}
						else if (closedList[x + newX][y + newY] == false)
						{
							gNew = node.gCost + 1.0;
							hNew = calculateH(x + newX, y + newY, dest); //add Caluclate H
                            oNew = calculateO(x + newX, y + newY, distances, params);  //add Caluculate O
							fNew = gNew + hNew + oNew; //add Obstacle Cost
							// Check if this path is better than the one already present
							if (allMap[x + newX][y + newY].fCost == FLT_MAX ||
								allMap[x + newX][y + newY].fCost > fNew)
							{
								// Update the details of this neighbour node
								allMap[x + newX][y + newY].fCost = fNew;
								allMap[x + newX][y + newY].gCost = gNew;
								allMap[x + newX][y + newY].hCost = hNew;
								allMap[x + newX][y + newY].parentX = x;
								allMap[x + newX][y + newY].parentY = y;
								openList.emplace_back(allMap[x + newX][y + newY]);
							}
						}
					}
				}
			}
		}
        
	if (destinationFound == false) 
	{
		cout << "Destination not found" << endl;
		return path;
	}

	//path.path_length = path.path.size();
	//return path;
}

inline bool operator < (const Node& lhs, const Node& rhs)
{//We need to overload "<" to put our struct into a set
    return lhs.fCost < rhs.fCost;
}

static bool isValid(int x, int y, const ObstacleDistanceGrid& distances, double minDist) 
{ //If our Node is an obstacle it is not valid
    if (distances.operator()(x,y) >= minDist) {
        if (x < 0 || y < 0 || x >= (distances.widthInCells()) || y >= (distances.heightInCells())) {
            return false;
        }
        return true;
    } 
   return false;
}

static bool isDestination(int x, int y, Node dest) 
{
    if (x == dest.x && y == dest.y) {
        return true;
    }
    return false;
}

static double calculateH(int x, int y, Node dest) 
{
    double H = (sqrt((x - dest.x)*(x - dest.x)
        + (y - dest.y)*(y - dest.y)));
    return H;
}

static double calculateO(int x, int y, const ObstacleDistanceGrid& distances, const SearchParams& params) 
{
    int dist = distances(x,y);
    if (dist >= params.maxDistanceWithCost)    return 0;
    else
    {
        ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
        ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
        return    pow(params.maxDistanceWithCost - dist, params.distanceCostExponent);
    }
    return 0.0;
}