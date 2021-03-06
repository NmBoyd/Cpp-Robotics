#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>

#include "FrenetOptimalPlanner.h"
#include "json.hpp"
#include "matplotlib-cpp/matplotlibcpp.h"

#include <chrono>
 
using namespace Eigen;
using namespace std::chrono;
namespace plt = matplotlibcpp;

int main()
{
    // waypoints
    std::vector<double> wp_x, wp_y;
    wp_x.push_back(0.0); wp_y.push_back(0.0);
    wp_x.push_back(10.0); wp_y.push_back(-6.0);
    wp_x.push_back(20.5); wp_y.push_back(5.0);
    wp_x.push_back(35.0); wp_y.push_back(6.5);
    wp_x.push_back(70.5); wp_y.push_back(0.0);
    wp_x.push_back(80.0); wp_y.push_back(5.0);
    wp_x.push_back(50.5); wp_y.push_back(7.0);
    wp_x.push_back(20.5); wp_y.push_back(0.0);

    wp_x.push_back(-10.5); wp_y.push_back(0.0);
    wp_x.push_back(-20.5); wp_y.push_back(5.0);
    wp_x.push_back(-20.0); wp_y.push_back(7.0);
    wp_x.push_back(0.5); wp_y.push_back(5.0);
    wp_x.push_back(0.0); wp_y.push_back(10.0);

    // obstacles
    // If the obstacle is too close to the goal, it will seg fault with nullptr from c_spline
    std::vector<double> ob_x, ob_y;
    ob_x.push_back(20.0); ob_y.push_back(10.0);
    ob_x.push_back(30.0); ob_y.push_back(6.0);
    ob_x.push_back(30.0); ob_y.push_back(8.0);
    ob_x.push_back(35.0); ob_y.push_back(8.0);
    ob_x.push_back(50.0); ob_y.push_back(3.0);
    ob_x.push_back(60.0); ob_y.push_back(5.0);
    ob_x.push_back(65.0); ob_y.push_back(-1.0);
    ob_x.push_back(15.0); ob_y.push_back(5.0);
    ob_x.push_back(10.0); ob_y.push_back(10.0);
    ob_x.push_back(20.0); ob_y.push_back(5.0);


    std::vector<Vector2d> obstacle_poses;
    for (int i=0;i<ob_x.size();i++)
    {
        obstacle_poses.push_back(Vector2d(ob_x[i], ob_y[i]));
    }

    std::cout << "Added obstacles and waypoint list" << std::endl;

    Spline2D route_c_spline(wp_x, wp_y);
    PathSE2 path;
    const double seg_len = 0.1; // segment length
    for (double i=0;i < route_c_spline.s.back(); i+=seg_len)
    {
        std::array<std::shared_ptr<double>, 2> point_ptr = route_c_spline.calc_postion(i);
        std::array<double, 2> point_;
        point_[0] = *point_ptr[0];
        point_[1] = *point_ptr[1];
        path.x.push_back(point_[0]);
        path.y.push_back(point_[1]);
        path.yaw.push_back(*route_c_spline.calc_yaw(i));
        path.k.push_back(*route_c_spline.calc_curvature(i));
    }

    std::cout << "Finished Constructing splines" << std::endl;

    int n = 500;
    // TODO: Write the frenet optimal planner
    // TODO: Tune where needed and add lane-based FSM
    FrenetOptimalPlanner::Params params;
    std::shared_ptr<FrenetOptimalPlanner> planner = std::make_shared<FrenetOptimalPlanner>();

    FrenetState curr_state;
    curr_state.speed = 10.0/3.6;    // current speed [m/s]
    curr_state.d = 0.0;             // current lateral position [m]
    curr_state.d_d = 0.0;           // current lateral speed [m/s]
    curr_state.d_dd = 0.0;          // current lateral accel [m/s2]
    curr_state.s = 1.0;             // current course position

    // from global x,y. Calc s. If s<0, draw a cubic spline to the initial position of the waypoint list
    // to prevent seg fault
	for(int i=0; i<n; i++) {

        // Calculate the optimal motion plan
        FrenetTrajectory traj;

        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        
        std::cout << "Attempting to calculate motion plan " << std::endl;

        traj = planner->calcOptimalMotionPlan(route_c_spline, curr_state, obstacle_poses);
        
        std::cout << "Calculated optimal trajectory" << std::endl;

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
        std::cout << "Calc Optimal Path Time: " << time_span.count() << " seconds.";
        std::cout << std::endl;
        
        // Update vehicle state (current, previous, and end state)
        curr_state.s = traj.s[1];
        curr_state.speed = traj.s_d[1];
        curr_state.d = traj.d[1];
        curr_state.d_d = traj.d_d[1];
        curr_state.d_dd = traj.d_dd[1];

        std::cout << "Current speed: " << curr_state.speed << " Goal Speed: " << planner->getParameters().target_speed<< std::endl;

        if (hypot(traj.global_path.x[1]-path.x.back(), traj.global_path.y[1]-path.y.back()) <= 1.0) {
            std::cout << "Goal Reached" << std::endl;
            break;
        }

        std::vector<double> x;
        x.push_back(traj.global_path.x[1]);
        std::vector<double> y;
        y.push_back(traj.global_path.y[1]);
        // Update nearby vehicle state

        // Update the route spline interpolation.

        // Frenet Optimal Planning (route spline, curr_course_pose, curr_speed, curr_lat_pose, curr_lat_speed, curr_lat_accel)
            // Calculate the list of frenet paths and convert them to global space
            // Check to make sure the paths are valid
            // find the minimum cost option
            // get the best path
        
        // Update the plot shown
        plt::clf();
        plt::scatter(wp_x, wp_y, 100.0);
        plt::scatter(ob_x, ob_y,200.0);
        plt::plot(path.x, path.y);
        plt::scatter(traj.global_path.x, traj.global_path.y, 50.0);
        plt::scatter(x, y, 100.0);
        plt::xlim(-100, 100);
        plt::title("World map");
        plt::legend();
        plt::grid(true);
        plt::pause(0.0001);
        std::cout << "looped" <<std::endl;
	}
    return 0;
}