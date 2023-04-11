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
     ////////////////// TODO: Implement your A* search here //////////////////////////
    mbot_lcm_msgs::robot_path_t path;
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    PriorityQueue Q;
    Node* curNode;
    Node* goalNode;
    Node* startNode;
    std::vector<Node*> neighbors;

    path.utime = start.utime;    
    path.path.push_back(start);
    path.path_length = path.path.size();

    goalNode->cell = goalCell;
    startNode->cell = startCell;
    curNode->cell = startCell;

    Q.push(curNode);

    
    while (!is_goal(curNode, goalNode)) { // if curNode == goalNode -> break
        curNode = Q.pop();
        neighbors = expand_node(curNode, distances, params);
        for (auto& neighbor : neighbors) {
            if (!is_in_list(neighbor, Q.elements)) {
                if (is_goal(neighbor, goalNode)){
                    curNode = neighbor;
                    std::vector<Node*> node_path = extract_node_path(curNode,startNode);
                    std::reverse(node_path.begin(),node_path.end());
                    
                    return path;
                }
                neighbor->g_cost = g_cost(neighbor, goalNode, distances, params);
                neighbor->h_cost = h_cost(neighbor, goalNode, distances);
                Q.push(neighbor);
            }
        }
    }

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
    g_cost = pow(params.maxDistanceWithCost - distances(from->cell.x, from->cell.y), params.distanceCostExponent);
    g_cost += from->parent->g_cost;
    if(from->cell.x == from->parent->cell.x || from->cell.y == from->parent->cell.y){
        g_cost += 1;
    }else{
        g_cost += 1.4;
    }

    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    std::vector<Node*> neighbors;
    Node* neighbor;
    const int xDeltas[8] = {1, 1, 1, 0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 1, -1, 0};

    double distAdd;

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
    Node* curr = goal_node;
    while (curr != start_node){
        path.push_back(curr);
        curr = curr->parent;
    }
    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
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
    collision = distances(node->cell.x, node->cell.y) < params.minDistanceToObstacle;
    return collision;
}

bool is_goal(Node* node, Node* goal){
    if(node->cell.x == goal->cell.x && node->cell.y == goal->cell.y){
        return true;
    }
    return false
}