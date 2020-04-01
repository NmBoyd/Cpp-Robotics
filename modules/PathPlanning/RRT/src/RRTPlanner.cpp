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
        // std::cout << "Extend length to large. " << extend_length <<" Setting to distance: " << dist_angle.first <<std::endl;
        extend_length = dist_angle.first;
    }
    // find how many node increments are needed to reach target node
    int n_expand_nodes = std::floor(extend_length/path_resolution_);

    // fill nodes up to the target node
    for (int i=0; i<n_expand_nodes; i++) {
        new_node->x_+= path_resolution_*std::cos(dist_angle.second);
        new_node->y_+= path_resolution_*std::sin(dist_angle.second);
        new_node->path_x_.push_back(new_node->x_);
        new_node->path_y_.push_back(new_node->y_);
    }

    std::pair<double,double> new_dist_angle = CalcDistanceAndAngle(new_node, target_node);

    // if there are rounding/resolution errors. Complete fill
    if (new_dist_angle.first <= path_resolution_) {
        new_node->path_x_.push_back(target_node->x_);
        new_node->path_y_.push_back(target_node->y_);
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
    return node_list_[min_id];
}

std::shared_ptr<Path> RRTPlanner::GenerateFinalPath()
{
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

std::shared_ptr<Path> RRTPlanner::GenerateFinalPath(cv::Mat bg, int img_reso)
{
    std::shared_ptr<Path> path = std::make_shared<Path>();
    path->x_.push_back(end_->x_);
    path->y_.push_back(end_->y_);
    Node* temp = node_list_.back();
    while (temp->parent_ != NULL) {
        cv::line(
            bg, 
            cv::Point((int)((temp->x_-rand_area_[0])*img_reso), (int)((temp->y_-rand_area_[0])*img_reso)), 
            cv::Point((int)((temp->parent_->x_-rand_area_[0])*img_reso), (int)((temp->parent_->y_-rand_area_[0])*img_reso)),
            cv::Scalar(255,0,255), 10);
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

std::shared_ptr<Path> RRTPlanner::Plan()
{
    #ifdef ANIMATE
        cv::namedWindow("rrt", cv::WINDOW_NORMAL);
        int count=0;
        int img_size = (int)(rand_area_[1] - rand_area_[0]);
        int img_reso = 50;
        cv::Mat bg(img_size * img_reso, img_size * img_reso,
                CV_8UC3, cv::Scalar(255,255,255));

        cv::circle(bg, cv::Point((int)((start_->x_-rand_area_[0])*img_reso), (int)((start_->y_-rand_area_[0])*img_reso)), 20, cv::Scalar(0,0,255), -1);
        cv::circle(bg, cv::Point((int)((end_->x_-rand_area_[0])*img_reso), (int)((end_->y_-rand_area_[0])*img_reso)), 20, cv::Scalar(255,0,0), -1);
        for(auto item:ob_list_){
        cv::circle(bg, cv::Point((int)((item.x_-rand_area_[0])*img_reso), (int)((item.y_-rand_area_[0])*img_reso)), (int)item.radius_ * img_reso, cv::Scalar(0,0,0), -1);
        }
    #endif

	node_list_.push_back(start_);
    for (int i=0; i<max_iter_;i++) 
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
        Node* rand_node = new Node(rand_pt.first, rand_pt.second);

        // std::cout << "Getting nearest node" << std::endl;
        Node* nearest_node = GetNearestNode(rand_pt);

        // std::cout << "Steering node " << std::endl;
        Node* new_node = Steer(nearest_node, rand_node, expand_dist_);

        // std::cout << "Checking collision" << std::endl;
        if (!CollisionCheck(new_node)) continue;
        
        // std::cout << "Adding node" << std::endl;
        node_list_.push_back(new_node);

        #ifdef ANIMATE
            cv::line(
                bg, 
                cv::Point((int)((new_node->x_-rand_area_[0])*img_reso), (int)((new_node->y_-rand_area_[0])*img_reso)), 
                cv::Point((int)((nearest_node->x_-rand_area_[0])*img_reso), (int)((nearest_node->y_-rand_area_[0])*img_reso)),
                cv::Scalar(0,255,0), 10);

            // std::string int_count = std::to_string(count);
            // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
            count++;
            cv::imshow("rrt", bg);
            cv::waitKey(5);
        #endif

        if (hypot(new_node->x_-end_->x_, new_node->y_-end_->y_) <= expand_dist_) {
            std::cout << "Path found" << std::endl;

            #ifdef ANIMATE
                std::shared_ptr<Path> path = GenerateFinalPath(bg, img_reso);
                cv::imshow("rrt", bg);
                cv::waitKey(2000);
            #else
                std::shared_ptr<Path> path = GenerateFinalPath();
            #endif 
            return path;

            break;
        }
    }
    return std::make_shared<Path>();
}