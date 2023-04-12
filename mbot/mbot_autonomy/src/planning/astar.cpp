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
    // std::cout << "a: " << startNode->cell.x << std::endl;

    startNode->cell = startCell;
    // std::cout << "start node: " << startNode->cell.x << ", " << startNode->cell.y << std::endl;

    nodeQueue.push(startNode);
    // std::cout << "d" << std::endl;

    // curNode = startNode;

    while (iterate && !nodeQueue.empty()) { // if curNode == goalNode -> break
        // std::cout << "iterating" << std::endl;
        curNode = nodeQueue.pop();
        visited.push_back(curNode);
        // std::cout << "curNode: " << curNode->cell.x << ", " << curNode->cell.y << std::endl;
        neighborNodes = expand_node(curNode, distances, params);
        if (neighborNodes.size() == 0) {
            iterate = false;
        }
        for (int i = 0; i < neighborNodes.size(); i++) {
            std::cout << neighborNodes[i]->cell.x << ", " << neighborNodes[i]->cell.y << std::endl;
        }
        std::cout << "visited: " << nodeQueue.Q.size() << std::endl;
        for (int i = 0; i < neighborNodes.size(); i++) {
            // std::cout << "neighbor: " << neighbor->cell.x << std::endl;
            if (!is_in_list(neighborNodes[i], visited) && !nodeQueue.is_member(neighborNodes[i])) {
                (*neighborNodes[i]).g_cost = g_cost(neighborNodes[i], goalNode, distances, params);
                (*neighborNodes[i]).h_cost = h_cost(neighborNodes[i], goalNode, distances);
                // std::cout << "h_cost: " << neighborNodes[i]->cell.x << ", " << neighborNodes[i]->cell.y << std::endl;
                nodeQueue.push(neighborNodes[i]);
                // std::cout << "Q size: " << nodeQueue.Q.size() << std::endl;
                if (neighborNodes[i]->cell.x == goalNode->cell.x && neighborNodes[i]->cell.y == goalNode->cell.y) {
                    goalNode = neighborNodes[i];
                    goalFound = true;
                    iterate = false;
                }

                // while (neighborNodes[i]->cell.x == 250 && neighborNodes[i]->cell.y == 150) {
                //     std::cout << "goal found!" << std::endl;
                // }
                // std::cout << "curNode: " << curNode->cell.x << ", " << curNode->cell.y << std::endl;
            } else {
                std::cout << "already in list!" << std::endl;
            }
        }
    }
    
    if (goalFound) {
        // std::cout << "extracting path" << std::endl;

        nodePath = extract_node_path(goalNode, startNode);

        // std::cout << "node path found" << std::endl;

        robotPath.path_length = nodePath.size();

        // std::cout << "length set" << std::endl;

        robotPath.path = extract_pose_path(nodePath, distances);

        // std::cout << "extracted pose path" << std::endl;
        // std::cout << std::endl;
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
        // std::cout << curNode->cell.x << ", " << curNode->cell.y << std::endl;
        if (curNode == start_node) {
            break;
        } else {
            // std::cout <<"attempt" << std::endl;
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
    mbot_lcm_msgs::pose_xyt_t curPose;
    Point<double> curPoint;
    Point<double> parentPoint;
    for (int i = 0; i < nodes.size(); i++) {
        curPoint = grid_position_to_global_position(Point<double>(nodes[i]->cell.x, nodes[i]->cell.y), distances);
        curPose.x = curPoint.x;
        curPose.y = curPoint.y;

        // std::cout<<curPose.x << ", " << curPose.y <<std::endl;

        if (i < nodes.size()-1 && i != 0) {
            // std::cout<<nodes[i-1]->cell.x << ", " << nodes[i-1]->cell.y<<std::endl;
            parentPoint = grid_position_to_global_position(Point<double>(nodes[i-1]->cell.x, nodes[i-1]->cell.y), distances);
            curPose.theta = atan2(parentPoint.y-curPoint.y, parentPoint.x-curPoint.x);
        } else {
            curPose.theta = 0;
        }

        path.push_back(curPose);
    }
    std::reverse(path.begin(), path.end());

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
    if (distances.isCellInGrid(node->cell.x, node->cell.y)) {
        collision = (distances(node->cell.x, node->cell.y) > params.minDistanceToObstacle && distances(node->cell.x, node->cell.y) < params.maxDistanceWithCost);
    } else {
        collision = true;
    }
    // std::cout << "collision: " << collision << std::endl;
    return collision;
}