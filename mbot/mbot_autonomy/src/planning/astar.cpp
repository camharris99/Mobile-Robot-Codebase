#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
                                             mbot_lcm_msgs::pose_xyt_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    ////////////////// TODO: Implement your A* search here //////////////////////////

    mbot_lcm_msgs::robot_path_t robotPath;
    std::vector<Node*> neighborNodes;
    std::vector<Node*> nodePath;
    PriorityQueue nodeQueue;
    std::vector<Node*> visited;
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    Node* curNode = new Node(startCell.x, startCell.y);
    bool iterate = true;
    bool goalFound = false;
    goalNode->cell = goalCell;

    startNode->cell = startCell;

    nodeQueue.push(startNode);

    while (iterate && !nodeQueue.empty()) {
        curNode = nodeQueue.pop();
        visited.push_back(curNode);

        neighborNodes = expand_node(curNode, distances, params);

        for (int i = 0; i < neighborNodes.size(); i++) {

            if (!is_in_list(neighborNodes[i], visited) && !is_in_list(neighborNodes[i], nodeQueue.elements)) {
                (*neighborNodes[i]).g_cost = g_cost(neighborNodes[i], goalNode, distances, params);
                (*neighborNodes[i]).h_cost = h_cost(neighborNodes[i], goalNode, distances);
                
                nodeQueue.push(neighborNodes[i]);

                if (neighborNodes[i]->cell.x == goalNode->cell.x && neighborNodes[i]->cell.y == goalNode->cell.y) {
                    goalNode = neighborNodes[i];
                    goalFound = true;
                    iterate = false;
                    break;
                }
            }
        }
    }
    
    if (goalFound) {
        nodePath = extract_node_path(goalNode, startNode);

        robotPath.path_length = nodePath.size();

        robotPath.path = extract_pose_path(nodePath, distances);
    }
    
    return robotPath;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    double h_cost = 0.0;
    double dx = std::fabs(from->cell.x - goal->cell.x);
    double dy = std::fabs(from->cell.y - goal->cell.y);
    h_cost = (dx+dy)+(1.414-2.0)*std::min(dx,dy);
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost
    double g_cost = 0.0;   
    double distAdd = 1.0;
    if (from->cell.x != from->parent->cell.x && from->cell.y != from->parent->cell.y) {
        distAdd = 1.4;
    }
    g_cost = pow(params.maxDistanceWithCost - distances(from->cell.x, from->cell.y), params.distanceCostExponent);
    g_cost += from->parent->g_cost;
    g_cost += distAdd;
    
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    std::vector<Node*> neighbors;
    const int xDeltas[8] = {1, 1, 1, 0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 1, -1, 0};

    for (int i = 0; i < 8; i++) {
        Node* neighbor = new Node(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
        neighbor->parent = node;
        if (!is_collision(neighbor, distances, params)) {
            neighbors.push_back(neighbor);
        } else {
            continue;
        }
    }

    return neighbors;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;

    Node* curNode = new Node(0,0);
    curNode = goal_node;

    while (true){
        path.push_back(curNode);
        if (curNode->cell.x == start_node->cell.x && curNode->cell.y == start_node->cell.y) {
            break;
        } else {
            curNode = curNode->parent;
        }
    }

    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    std::vector<mbot_lcm_msgs::pose_xyt_t> temp_path;
    Point<double> curPoint;
    Point<double> parentPoint;
    for (int i = 0; i < nodes.size(); i++) {
        mbot_lcm_msgs::pose_xyt_t curPose = {0, 0.0, 0.0, 0.0};
        curPoint = grid_position_to_global_position(Point<double>(nodes[i]->cell.x, nodes[i]->cell.y), distances);
        curPose.x = curPoint.x;
        curPose.y = curPoint.y;

        if (i < nodes.size() - 1 && i != 0) {
            parentPoint = grid_position_to_global_position(Point<double>(nodes[i-1]->cell.x, nodes[i-1]->cell.y), distances);
            curPose.theta = atan2(parentPoint.y-curPoint.y, parentPoint.x-curPoint.x);
        } else {
            curPose.theta = 0;
        }

        temp_path.push_back(curPose);
    }

    std::reverse(temp_path.begin(), temp_path.end());

    path.push_back(temp_path[0]);

    for (int i = 3; i < temp_path.size() - 2; i+=3) {
        path.push_back(temp_path[i]);
    }

    path.push_back(temp_path[temp_path.size()-1]);

    return path;

}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;   
}

bool is_collision(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params) {
    bool collision;
    int factor = 1;
    if (distances.isCellInGrid(node->cell.x, node->cell.y)) {
        for (int i = 2; i < 10; i++) {
            if (distances(node->cell.x, node->cell.y) > factor*params.maxDistanceWithCost) {
                factor = i;
            } else {
                break;
            }
        }
        
        collision = !(distances(node->cell.x, node->cell.y) > params.minDistanceToObstacle && 
                    ((distances(node->cell.x, node->cell.y) < factor*params.maxDistanceWithCost) || distances(node->cell.x, node->cell.y) == INT32_MAX));
    } else {
        collision = true;
    }

    return collision;
}