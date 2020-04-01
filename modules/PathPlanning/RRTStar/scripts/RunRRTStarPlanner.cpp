#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>

#include "json.hpp"
#include "matplotlib-cpp/matplotlibcpp.h"
#include "RRTStarPlanner.h"
 
using namespace Eigen;
using namespace std::chrono;
namespace plt = matplotlibcpp;

int main(){
    // x, y, radius
	std::vector< Obstacle > obstacle_list;
    obstacle_list.push_back(Obstacle{5, 5, 2});
    obstacle_list.push_back(Obstacle{3, 6, 2});
    obstacle_list.push_back(Obstacle{3, 8, 2});
    obstacle_list.push_back(Obstacle{3, 10, 2});
    obstacle_list.push_back(Obstacle{7, 5, 2});
    obstacle_list.push_back(Obstacle{9, 5, 2});

	Node* start = new Node(0.0, 0.1);
	Node* goal = new Node(6.0, 9.0);
	std::vector<double> rand_area{-2, 15};

    std::vector<double> end_pts_x, end_pts_y;
    end_pts_x.push_back(start->x_); end_pts_y.push_back(start->y_);
    end_pts_x.push_back(goal->x_); end_pts_y.push_back(goal->y_);

    std::cout << "Creating planner" << std::endl;
	// RRTStarPlanner rrt_star(start, goal, obstacle_list, rand_area, 0.5, 1.0, 5, 500, 50.0);
    RRTStarPlanner rrt_star(start, goal, obstacle_list, rand_area, 1);


    std::cout << "Trying to find path" << std::endl;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

	std::shared_ptr<Path> path = rrt_star.Plan();

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "Calculated path in: " << time_span.count() << " seconds" << std::endl;
    std::cout << "Calculated path" << std::endl;

    for (int i=0;i<path->x_.size();i++) {
        std::cout << "Point: " << i << " at <"<<path->x_[i]<<","<<path->y_[i]<<">"<<std::endl;
    }

}
