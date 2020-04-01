#pragma once
#include <iostream> 
#include <vector>

class Node {
public:
    Node(int x, int y, double cost=0.0, Node* prev_node=NULL);

    std::string asString();
public:
    int x_; // grid index
    int y_; // grid index
    double cost_;
    Node* prev_node_;
};