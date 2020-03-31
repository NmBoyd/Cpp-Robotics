#include "RRTPlanner.h"

std::pair<double, double> RRTPlanner::CalcDistanceAndAngle(Node* from_node, Node* target_node)
{
    double dx = (target_node->x_-from_node->x_);
    double dy = (target_node->y_-from_node->y_);
    double dist = hypot(dx, dy);
    double theta = std::atan2(dy,dx);
    return std::make_pair(dist, theta);
}

Node* RRTPlanner::Steer(Node* from_node, Node* target_node, double extend_length)
{
    Node* new_node = new Node(from_node->x_, from_node->y_);
    // find the distance and angle to the target_node
    std::pair<double,double> dist_angle = CalcDistanceAndAngle(new_node, target_node);
    new_node->path_x_.push_back(new_node->x_);
    new_node->path_y_.push_back(new_node->y_);

    if (extend_length > dist_angle.first) {
        extend_length - dist_angle.first;
    }
    // find how many node increments are needed to reach target node
    int expand_node = std::floor(extend_length/path_resolution_);

    // fill nodes up to the target node
    for (int i=0; i<expand_node; i++) {
        new_node->x_+= path_resolution_*std::cos(dist_angle.first);
        new_node->y_+= path_resolution_*std::sin(dist_angle.second);
        new_node->path_x_.push_back(new_node->x_);
        new_node->path_y_.push_back(new_node->y_);
    }

    std::pair<double,double> new_dist_angle = CalcDistanceAndAngle(new_node, target_node);

    // if there are rounding/resolution errors. Complete fill
    if (new_dist_angle.first <= path_resolution_) {
        new_node->x_ = target_node->x_;
        new_node->y_ = target_node->y_;
        new_node->path_x_.back()=target_node->x_;
        new_node->path_y_.back()=target_node->y_;
    }
    new_node->parent_ = from_node;

    return new_node;
}

bool RRTPlanner::CollisionCheck(Node* node) {
    for (Obstacle item : ob_list_) {
        if (hypot((item.x_-node->x_), (item.y_-node->y_)) <= item.radius_) {
            return false;
        }
    }
    return true;
}

Node* RRTPlanner::GetNearestNode(const std::pair<double,double> rand_pt) 
{
    int min_id = -1;
    double min_dist = std::numeric_limits<double>::max();

    // search through the node list for the closest point
    for (int i=0; i<node_list_.size(); i++) {
        double dist = hypot(node_list_[i]->x_-rand_pt.first, node_list_[i]->y_-rand_pt.second);
        if (dist < min_dist) {
            min_dist = dist;
            min_id = i;
        }
    }
    std::cout << "closest node index " << min_id << std::endl; 
    return node_list_[min_id];
}

std::shared_ptr<Path> RRTPlanner::Plan()
{
	node_list_.push_back(start_);
    while (true) 
    {
        std::pair<double,double> rand_pt;
        // std::cout << "Generating random point" << std::endl;
        if (goal_dis_(goal_gen_)>goal_sample_rate_)
        {
            double rand_x = area_dis_(goal_gen_);
            double rand_y = area_dis_(goal_gen_);
            rand_pt = std::make_pair(rand_x, rand_y);
        }
        else {
            rand_pt = std::make_pair(end_->x_, end_->y_);
        }

        // std::cout << "Getting nearest node" << std::endl;
        Node* nearest_node = GetNearestNode(rand_pt);

        float theta = std::atan2(rand_pt.second-nearest_node->y_,rand_pt.first-nearest_node->x_);

        // std::cout << "Creating new node" << std::endl;
        Node* new_node = new Node(nearest_node->x_+expand_dis_*std::cos(theta),
                                  nearest_node->y_+expand_dis_*std::sin(theta));
        new_node->parent_ = nearest_node;

        // std::cout << "Checking collisions" << std::endl;
        if (!CollisionCheck(new_node)) continue;
        
        // std::cout << "Adding node" << std::endl;
        node_list_.push_back(new_node);

        if (hypot(new_node->x_-end_->x_, new_node->y_-end_->y_) <= expand_dis_) {
            std::cout << "Path found" << std::endl;
            break;
        }
    }

    std::shared_ptr<Path> path = std::make_shared<Path>();
    path->x_.push_back(end_->x_);
    path->y_.push_back(end_->y_);
    Node* temp = node_list_.back();
    while (temp->parent_ != NULL) {
        path->x_.push_back(temp->x_);
        path->y_.push_back(temp->y_);
        temp = temp->parent_;
    }
    path->x_.push_back(start_->x_);
    path->y_.push_back(start_->y_);
    std::reverse(path->x_.begin(), path->x_.end());
    std::reverse(path->y_.begin(), path->y_.end());
    return path;
}