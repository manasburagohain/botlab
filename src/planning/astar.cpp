#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{

	
	/*
    ////////////////// TODO: Implement your A* search here //////////////////////////
    Node start, dest;
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();
    return path;

    //convert to grid cell
    dest.x = grid_position_to_global_position
    dest.y = 
    start.x = 
    start.y = 
    if (!isValid(dest.x, dest.y, distances, params.minDistanceToObstacle))
    {
        print("Destination is cannot be reached");
        return -1;
    }    

    if(isDestination(start.x, start.y, dest)
    {
        print("Already at Destination");
        return path;
    }

    bool closedList[XGRID][YGRID];
    array<array < Node, (YGRID)>, XGRID> allMap;
    for (int x = 0; x < (XGRID); x++) 
    {
			for (int y = 0; y < (YGRID); y++) {
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
    
    int x = start.x;
	int y = start.y;
	allMap[x][y].fCost = 0.0;
	allMap[x][y].gCost = 0.0;
	allMap[x][y].hCost = 0.0;
	allMap[x][y].parentX = x;
	allMap[x][y].parentY = y;
    
    vector<Node> openList;	
	openList.emplace_back(allMap[x][y]);
	bool destinationFound = false;

    while (!openList.empty()&&openList.size()<(XGRID)*(YGRID)) 
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
							return makePath(allMap, dest);
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
            return empty;
		}
		*/
	robot_path_t path;
	return path;
}
