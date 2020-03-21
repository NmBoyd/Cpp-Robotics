#include <iostream>
#include <vector>
#include <math.h>
#include <memory>
#include <algorithm>
#include <cassert>
#include <map>
#include <queue>
#include <limits>

#include "Node.h"

// Use (void) to silent unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

struct Path {
    std::vector<double> x;
    std::vector<double> y;
};

struct OccupancyMap {
    double min_x_;
    double min_y_;
    double max_x_;
    double max_y_;

    int x_width_;
    int y_width_;
    std::vector<std::vector<bool>> map_;
};

class AStarPlanner {
public:
    AStarPlanner(std::vector<double> o_x, 
        std::vector<double> o_y, 
        double grid_size, 
        double robot_radius);

    std::shared_ptr<Path> Plan(double start_x, double start_y, double goal_x, double goal_y);

public:
    double resolution_;
    double robot_radius_;

    OccupancyMap occupancy_map_;

private:

    std::shared_ptr<Path> CalcFinalPath(Node* goal);
    double CalcHeuristic(Node* n1, Node* n2, double w = 1.0);
    double CalcGridPosition(int node_index, double min_position);
    int CalcXYIndex(double position, double min_position);
    double CalcGridIndex(Node node);
    bool VerifyNode(Node* node);
    std::shared_ptr<OccupancyMap> CreateOccupancyMap(std::vector<double> o_x, std::vector<double> o_y);
    std::vector<Node> GetMotionModel();
};