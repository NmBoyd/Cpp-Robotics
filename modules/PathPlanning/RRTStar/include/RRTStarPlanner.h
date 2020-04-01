#include <iostream>
#include <limits>
#include <random>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include "RRTPlanner.h"

// #define ANIMATE

// Use (void) to silent unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

class RRTStarPlanner : public RRTPlanner{
public:

    RRTStarPlanner(Node* start_node, Node* goal_node, 
                   std::vector<Obstacle> obstacle_list, 
                   std::vector<double> rand_area, double expand_dist=1.0, 
                   double path_resolution=0.5, int goal_sample_rate=5, 
                   int max_iterations=500, double connect_circle_dist=50)
        : RRTPlanner(start_node, goal_node,
                    obstacle_list,
                    rand_area, expand_dist,
                    path_resolution, goal_sample_rate,
                    max_iterations), 
          connect_circle_dist_(connect_circle_dist){};

    std::shared_ptr<Path> Plan();

private:

    std::vector<Node*> FindNearNodes(Node* new_node);

    double CalcNewCost(Node* start_node, Node* target_node);

    Node* ChooseParent(Node* node, std::vector<Node*> near_nodes);

    void PropagateCostToLeaves(Node* parent_node);

    void Rewire(Node* new_node, std::vector<Node*> neighbor_nodes);

private:
    double connect_circle_dist_;

};