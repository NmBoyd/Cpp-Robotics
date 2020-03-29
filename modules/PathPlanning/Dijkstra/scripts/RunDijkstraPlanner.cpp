#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>

#include "json.hpp"
#include "matplotlib-cpp/matplotlibcpp.h"
#include "DijkstraPlanner.h"
#include <chrono>
 
using namespace Eigen;
using namespace std::chrono;
namespace plt = matplotlibcpp;

int main()
{
    // waypoints
    double s_x, s_y; //start position [m]
    double g_x, g_y; //goal position [m]
    s_x = 10.0;
    s_y = 10.0;
    g_x = 50.0;
    g_y = 50.0;
    
    double grid_size = 1.0; // [m]
    double robot_radius = 2.0; // [m]

    // Occupied spaces
    std::vector<double> ob_x, ob_y;
    for (int i = -10; i < 60; i++) {
        ob_x.push_back(i);
        ob_y.push_back(-10.0);
    }
    for (int i = -10; i < 60; i++) {
        ob_x.push_back(60);
        ob_y.push_back(i);
    }
    for (int i = -10; i < 61; i++) {
        ob_x.push_back(i);
        ob_y.push_back(60.0);
    }
    for (int i = -10; i < 61; i++) {
        ob_x.push_back(-10.0);
        ob_y.push_back(i);
    }
    for (int i = -10; i < 40; i++) {
        ob_x.push_back(20.0);
        ob_y.push_back(i);
    }
    for (int i = 0; i < 40; i++) {
        ob_x.push_back(40.0);
        ob_y.push_back(60.0-i);
    }

    std::cout << "Creating planner" << std::endl;
    DijkstraPlanner dijkstra(ob_x, ob_y, grid_size, robot_radius);

    std::cout << "Trying to find path" << std::endl;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    std::shared_ptr<Path> path = dijkstra.Plan(s_x, s_y, g_x, g_y);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "Calculated path in: " << time_span.count() << " seconds" << std::endl;
    plt::clf();
    plt::scatter(ob_x, ob_y,200.0);
    plt::plot(path->x, path->y);
    plt::xlim(-20, 70);
    plt::title("World map");
    plt::legend();
    plt::grid(true);
    plt::show();
	
    return 0;
}