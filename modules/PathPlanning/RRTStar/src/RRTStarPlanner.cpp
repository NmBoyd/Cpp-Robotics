#include "RRTStarPlanner.h"

Node* RRTStarPlanner::ChooseParent(Node* new_node, std::vector<Node*> neighbors)
{
    Node* output;
    if (neighbors.size()==0) {
        std::cout << "No neighbors found to determine parent" << std::endl;
        return output;
    }
    std::vector<double> costs;
    for (Node* near_node: neighbors) {
        Node* t_node = Steer(near_node, new_node);
        if (t_node != NULL && CollisionCheck(t_node)) {
            costs.push_back(CalcNewCost(near_node, new_node));
        }
        else {
            // Cost of collision
            costs.push_back(std::numeric_limits<double>::max()); 
        }
    }
    double min_cost = std::numeric_limits<double>::max();
    Node* min_cost_node;
    std::cout << "Costs: ";
    for (int i=0; i<costs.size(); i++) {
        std::cout << costs[i] << ", ";  
        if (costs[i] < min_cost && costs[i] != 1) {
            min_cost = costs[i];
            min_cost_node = node_list_[i];
        }
    }
    std::cout <<""<< std::endl;

    std::cout <<"Min Cost Parent Node: <" << min_cost_node->x_ <<","<<min_cost_node->y_<<"> Cost: " << min_cost << std::endl;  
    if (min_cost == std::numeric_limits<double>::max()) {
        std::cout << "There is no valid path. Min cost = inf" << std::endl;
        return output;
    }

    output = Steer(min_cost_node, new_node);
    output->parent_ = min_cost_node;
    output->cost_ = min_cost;

    return output;
}

double RRTStarPlanner::CalcNewCost(Node* from_node, Node* target_node)
{
    double distance = CalcDistanceAndAngle(from_node, target_node).first;
    return from_node->cost_+distance;
}

std::vector<Node*> RRTStarPlanner::FindNearNodes(Node* new_node)
{
    std::vector<Node*> output;
    int num_nodes = node_list_.size()+1;
    double r = connect_circle_dist_*std::sqrt(std::log(num_nodes)/num_nodes);
    if (r<expand_dist_ && expand_dist_!=std::numeric_limits<double>::max()) {
        r = expand_dist_;
    }
    // if expand_dist exists, search vertices in a range no more than expand_dist
    for (Node* n : node_list_) 
    {
        if (hypot((n->x_-new_node->x_), (n->y_-new_node->y_)) <= r) {
            output.push_back(n);
            // std::cout << "Nearby node: " << n->x_<<","<<n->y_<<std::endl;
        }
    }
    return output;
}

void RRTStarPlanner::PropagateCostToLeaves(Node* parent_node)
{
    for (Node* node : node_list_)
    {
        if (node->parent_ == parent_node) {
            node->cost_ = CalcNewCost(parent_node, node);
            PropagateCostToLeaves(node);
        }
    }
}

void RRTStarPlanner::Rewire(Node* new_node, std::vector<Node*> neighbor_nodes)
{
    for (int i=0; i < neighbor_nodes.size(); i++) 
    {
        Node* edge_node = Steer(new_node, neighbor_nodes[i]);
        if (neighbor_nodes[i] == NULL) continue;

        edge_node->cost_ = CalcNewCost(new_node, neighbor_nodes[i]);
        if (CollisionCheck(neighbor_nodes[i]) && 
            neighbor_nodes[i]->cost_ > edge_node->cost_)
        {
            node_list_[i] = edge_node;
            PropagateCostToLeaves(new_node);
        } 
    }
}

std::shared_ptr<Path> RRTStarPlanner::Plan()
{
    #ifdef ANIMATE
        cv::namedWindow("rrt_star", cv::WINDOW_NORMAL);
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
    for (int i=0;i<max_iter_;i++)
    {
        std::cout << "Iter:"<< i<< ", number of nodes:"<< node_list_.size()<<std::endl;
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

        std::cout << "Determining nearest node and parent" << std::endl;
        std::vector<Node*> neighbor_nodes = FindNearNodes(new_node);
        new_node = ChooseParent(new_node, neighbor_nodes);
        node_list_.push_back(new_node);

        std::cout << "Rewiring" << std::endl;
        Rewire(new_node, neighbor_nodes);
        
        #ifdef ANIMATE
            cv::line(
                bg, 
                cv::Point((int)((new_node->x_-rand_area_[0])*img_reso), (int)((new_node->y_-rand_area_[0])*img_reso)), 
                cv::Point((int)((nearest_node->x_-rand_area_[0])*img_reso), (int)((nearest_node->y_-rand_area_[0])*img_reso)),
                cv::Scalar(0,255,0), 10);

            // std::string int_count = std::to_string(count);
            // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
            count++;
            cv::imshow("rrt_star", bg);
            cv::waitKey(5);
        #endif

        if (hypot(new_node->x_-end_->x_, new_node->y_-end_->y_) <= expand_dist_) {
            std::cout << "Path found" << std::endl;
            // break;
        }
    }

    #ifdef ANIMATE
        std::shared_ptr<Path> path = GenerateFinalPath(bg, img_reso);
        cv::imshow("rrt_star", bg);
        cv::waitKey(4000);
    #else
        std::shared_ptr<Path> path = GenerateFinalPath();
    #endif 

    return path;
}