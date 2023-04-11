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

    mbot_lcm_msgs::robot_path_t path;
    std::vector<Node*> neighbors;
    std::vector<Node*> nodePath;
    PriorityQueue Q;
    Node* startNode;
    Node* goalNode;
    Node* curNode;
    bool iterate;

    goalNode->cell = goalCell;

    path.utime = start.utime;    
    path.path.push_back(start);
    path.path_length = path.path.size();

    startNode->cell = startCell;
    startNode->h_cost = 0;
    startNode->g_cost = 0;
    startNode->parent = NULL;

    Q.push(startNode);

    curNode = startNode;

    iterate = true;

    while (iterate) { // if curNode == goalNode -> break
        curNode = Q.pop();
        neighbors = expand_node(curNode, distances, params);
        for (auto& neighbor : neighbors) {
            if (!is_in_list(neighbor, Q.elements)) {
                neighbor->g_cost = g_cost(neighbor, goalNode, distances, params);
                neighbor->h_cost = h_cost(neighbor, goalNode, distances);
                Q.push(neighbor);
                if (neighbor == goalNode) {
                    iterate = false;
                }
            }
        }
    }

    nodePath = extract_node_path(goalNode, startNode);
    
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    double h_cost = 0;
    double dx = std::fabs(from->cell.x - goal->cell.x);
    double dy = std::fabs(from->cell.y - goal->cell.y);
    h_cost = (dx+dy)+(1.414-2.0)*std::min(dx,dy);
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost
    double g_cost = 0;   
    double distAdd = 1;
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
    Node* neighbor;
    const int xDeltas[8] = {1, 1, 1, 0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 1, -1, 0};

    for (int i = 0; i < 8; i++) {
        neighbor->cell.x = node->cell.x + xDeltas[i];
        neighbor->cell.y = node->cell.y + yDeltas[i];
        neighbor->parent = node;
        if (!is_collision(neighbor, distances, params)) {
                neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;

    Node* curNode = goal_node;

    do {
        path.push_back(curNode);
        curNode = curNode->parent;
    } while (curNode != start_node);

    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    std::reverse(nodes.begin(), nodes.end());
    mbot_lcm_msgs::pose_xyt_t curPose;
    Point<double> curPoint;
    Point<double> parentPoint;
    for (auto& node : nodes) {
        curPoint = grid_position_to_global_position(Point<double>(node->cell.x, node->cell.y), distances);
        curPose.x = curPoint.x;
        curPose.y = curPoint.y;

        if (node-> parent != NULL) {
            parentPoint = grid_position_to_global_position(Point<double>(node->parent->cell.x, node->parent->cell.y), distances);
            curPose.theta = atan2(parentPoint.y-curPoint.y, parentPoint.x-curPoint.x);
        } else {
            curPose.theta = 0;
        }

        path.push_back(curPose);
    }
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
    collision = distances(node->cell.x, node->cell.y) < params.minDistanceToObstacle && distances(node->cell.x, node->cell.y) < params.maxDistanceWithCost;
    return collision;
}