#include "DijkstraPlanner.h"

DijkstraPlanner::DijkstraPlanner(std::vector<double> o_x,
    std::vector<double> o_y,
    double grid_size,
    double robot_radius)
    : resolution_(grid_size), robot_radius_(robot_radius)
{
    occupancy_map_ = *CreateOccupancyMap(o_x, o_y);
}

std::shared_ptr<Path> DijkstraPlanner::Plan(double start_x, double start_y, double goal_x, double goal_y)
{
    std::cout << "Creating nodes" << std::endl;
    Node* start_node = new Node(CalcXYIndex(start_x, occupancy_map_.min_x_), 
                                CalcXYIndex(start_y, occupancy_map_.min_y_),
                                0.0);
    Node* goal_node = new Node(CalcXYIndex(goal_x, occupancy_map_.min_x_),
                               CalcXYIndex(goal_y, occupancy_map_.min_y_),
                               0.0);
    std::cout << "Creating visit map" << std::endl;
    std::cout << occupancy_map_.x_width_ <<" "<< occupancy_map_.y_width_<< std::endl;
    // Visited squares
    std::vector<std::vector<int> > visit_map(occupancy_map_.x_width_, std::vector<int>(occupancy_map_.y_width_, 0));
    std::cout << "Creating cost path" << std::endl;
    // Cost of a grid square
    std::vector<std::vector<double> > path_cost(occupancy_map_.x_width_, 
                                              std::vector<double>(occupancy_map_.y_width_,
                                                                 std::numeric_limits<double>::max()
                                                                ));
    path_cost[start_node->x_][start_node->y_] = 0;

    std::cout << "Creating node queue" << std::endl;
    // Create a priority queue of nodes
    // NOTE: d_ary_heap should be a better choice here
    auto cmp = [](const Node* left, const Node* right){return left->cost_ > right->cost_;};
    
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> pq(cmp);
    pq.push(start_node);
    std::vector<Node> motion = GetMotionModel();
    
    while (true) 
    {

        std::cout << "Grabbing node at top of the queue" << std::endl;
        Node* node = pq.top();
        
        if (visit_map[node->x_][node->y_] == 1) {
            pq.pop();
            std::cout << "removing node. Area explored" << std::endl;
            delete node;
            continue;
        }
        else {
            pq.pop();
            visit_map[node->x_][node->y_] = 1;
        }
        
        if (node->x_ == goal_node->x_ && node->y_ == goal_node->y_) {
            std::cout << "Reached goal" << std::endl;
            goal_node->cost_ = node->cost_;
            goal_node->prev_node_ = node;
            std::cout << "Cost: " << goal_node->cost_ << " Prev Node: " << goal_node->prev_node_->x_ <<" "<<goal_node->prev_node_->y_ << std::endl;
            break;
        }

        for (int i=0; i<motion.size(); i++)
        {
            std::cout << "Expanding with motion: "<< i << std::endl;
            // Make a new node around the current node
            // Calculate the cost based on the total path cost, motion cost, and heuristic (distance) to the goal
            Node* new_node = new Node(node->x_+motion[i].x_,
                                      node->y_+motion[i].y_,
                                      path_cost[node->x_][node->y_]+motion[i].cost_,
                                      node);

            if (!VerifyNode(new_node)) {
                delete new_node;
                continue;
            }
            if (visit_map[new_node->x_][new_node->y_]) {
                delete new_node;
                continue;
            }

            // Draw stuff

            // If the cost of the path with new motion is best, use it
            if (path_cost[node->x_][node->y_]+motion[i].cost_ < path_cost[new_node->x_][new_node->y_])
            {
                path_cost[new_node->x_][new_node->y_]=path_cost[node->x_][node->y_]+motion[i].cost_; 
                pq.push(new_node);
            }
        }
    }
    std::shared_ptr<Path> path = CalcFinalPath(goal_node);
    delete goal_node;
    delete start_node;
    return path;
}

std::shared_ptr<Path> DijkstraPlanner::CalcFinalPath(Node* goal_node)
{
    std::cout<< "Calculate the final route" << std::endl;
    std::cout << "Cost: " << goal_node->cost_ << " Prev Node: " << goal_node->prev_node_->x_ <<" "<<goal_node->prev_node_->y_ << std::endl;
    std::shared_ptr<Path> path = std::make_shared<Path>();
    Node* node = goal_node;
    while (node->prev_node_ != NULL)
    {
        node = node->prev_node_;
        path->x.push_back(CalcGridPosition(node->x_, occupancy_map_.min_x_));
        path->y.push_back(CalcGridPosition(node->y_, occupancy_map_.min_y_));
    }
    return path;
}

double DijkstraPlanner::CalcGridPosition(int node_index, double min_position)
{
    double position = node_index * resolution_ + min_position;
    std::cout << "Position " << position << std::endl;
    return position; 
}

int DijkstraPlanner::CalcXYIndex(double position, double min_position)
{
    return round((position-min_position)/resolution_);
}

double DijkstraPlanner::CalcGridIndex(Node node)
{
    return (node.y_-occupancy_map_.min_y_)*occupancy_map_.x_width_+(node.x_-occupancy_map_.min_x_);
}

bool DijkstraPlanner::VerifyNode(Node* node)
{
    double pos_x = CalcGridPosition(node->x_, occupancy_map_.min_x_);
    double pos_y = CalcGridPosition(node->y_, occupancy_map_.min_y_);
    
    // check limits
    if (pos_x < occupancy_map_.min_x_ ||
        pos_y < occupancy_map_.min_y_) {
        return false;
    } 
    else if (pos_x >= occupancy_map_.max_x_ ||
            pos_y >= occupancy_map_.max_y_) {
        return false;
    }

    // Collision if the space is occupied
    if (occupancy_map_.map_[node->x_][node->y_]) {
        return false;
    }
    return true;
}

std::shared_ptr<OccupancyMap> DijkstraPlanner::CreateOccupancyMap(std::vector<double> o_x, std::vector<double> o_y)
{
    assertm(o_x.size()==o_y.size(), "Can't fill occupancy grid. Occupancy positions are not the same size. There are N x and M y values");
    std::shared_ptr<OccupancyMap> occupancy_map = std::make_shared<OccupancyMap>();
    occupancy_map->min_x_ = round(*std::min_element(o_x.begin(), o_x.end()));
    occupancy_map->min_y_ = round(*std::min_element(o_y.begin(), o_y.end()));
    occupancy_map->max_x_ = round(*std::max_element(o_x.begin(), o_x.end()));
    occupancy_map->max_y_ = round(*std::max_element(o_y.begin(), o_y.end()));
    std::cout << "min_x: " << occupancy_map->min_x_
            << " min_y: " << occupancy_map->min_y_
            << " max_x: " << occupancy_map->max_x_
            << " max_y: " << occupancy_map->max_y_ << std::endl;

    occupancy_map->x_width_ = round((occupancy_map->max_x_-occupancy_map->min_x_)/resolution_);
    occupancy_map->y_width_ = round((occupancy_map->max_y_-occupancy_map->min_y_)/resolution_);
    std::cout << " x_width: " << occupancy_map->x_width_
            << " y_width: " << occupancy_map->y_width_ << std::endl;

    occupancy_map->map_.resize(occupancy_map->y_width_, std::vector<bool>(occupancy_map->x_width_, false));
    for (int ix=0; ix<occupancy_map->x_width_; ix++) {
        double x = CalcGridPosition(ix, occupancy_map->min_x_);
        for (int iy=0; iy<occupancy_map->y_width_; iy++) {
            double y = CalcGridPosition(iy, occupancy_map->min_y_);
            for (int i_ob=0; i_ob<o_x.size(); i_ob++) {
                double dist = hypot(o_x[i_ob]-x, o_y[i_ob]-y);
                if (dist <= robot_radius_) {
                    occupancy_map->map_[ix][iy] = true;
                    break;
                }
            }
        }
    }
    return occupancy_map;
}

std::vector<Node> DijkstraPlanner::GetMotionModel()
{
    // dx, dy, cost
    return {Node(1, 0, 1),          // "east"
            Node(0, 1, 1),          // "north"
            Node(-1, 0, 1),         // "west"
            Node(0, -1, 1),         // "south"
            Node(-1, -1, sqrt(2)),  // "southwest"
            Node(-1, 1, sqrt(2)),   // "northwest"
            Node(1, -1, sqrt(2)),   // "southeast"
            Node(1, 1, sqrt(2))     //"northeast"
        };
}
