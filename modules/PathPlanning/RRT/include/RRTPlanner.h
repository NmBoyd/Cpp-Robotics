#include <iostream>
#include <limits>
#include <random>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

#include "Node.h"

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#define ANIMATE

// Use (void) to silent unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

struct Path {
    std::vector<double> x_;
    std::vector<double> y_;
};

struct Obstacle{
    double x_;
    double y_;
    double radius_;
};

class RRTPlanner{
public:
    RRTPlanner(Node* start_node, Node* goal_node,
               std::vector< Obstacle > obstacle_list,
               std::vector<double> rand_area, double expand_dist=1.0,
               double path_resolution=0.5, int goal_sample_rate=5,
               double max_iterations=500)
        : start_(start_node), end_(goal_node), ob_list_(obstacle_list), 
        expand_dist_(expand_dist), path_resolution_(path_resolution), 
        goal_sample_rate_(goal_sample_rate), max_iter_(max_iterations), 
        goal_gen_(goal_rd_()), goal_dis_(std::uniform_int_distribution<int>(0, 100)), 
        area_gen_(area_rd_()), area_dis_(std::uniform_real_distribution<double>(rand_area_[0], rand_area_[1])), 
        rand_area_(rand_area){};
 ;

    std::shared_ptr<Path> Plan();

    Node* GetNearestNode(const std::pair<double,double> rand_pt);

    bool CollisionCheck(Node* node);

protected:
    Node* start_;
	Node* end_;
	const double expand_dist_;
	const double path_resolution_;
	const int goal_sample_rate_;
	const int max_iter_;
	const std::vector<Obstacle> ob_list_;

	std::vector<double> rand_area_;
	std::vector<Node*> node_list_;

	std::random_device goal_rd_;
    std::mt19937 goal_gen_;
    std::uniform_int_distribution<int> goal_dis_;
	
	std::random_device area_rd_;
    std::mt19937 area_gen_;
    std::uniform_real_distribution<double> area_dis_;

protected:

    std::pair<double, double> CalcDistanceAndAngle(Node* from_node, Node* target_node);
    
    Node* Steer(Node* from_node, Node* target_node, double extend_length=std::numeric_limits<double>::max());

    std::shared_ptr<Path> GenerateFinalPath();
    std::shared_ptr<Path> GenerateFinalPath(cv::Mat bg, int img_reso);

};