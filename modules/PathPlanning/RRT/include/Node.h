#pragma once
#include <vector>
#include <iostream>

class Node {
public:
    Node(double x, double y, double cost=0.0, Node* parent=NULL) 
    : x_(x), y_(y), cost_(cost), parent_(parent) {};

    std::string asString();
public:
    double x_; 
    double y_; 
    double cost_;
    Node* parent_;

    std::vector<double> path_x_;
    std::vector<double> path_y_;
};