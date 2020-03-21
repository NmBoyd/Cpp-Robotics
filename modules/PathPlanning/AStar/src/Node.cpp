#include "Node.h"

Node::Node(int x, int y, double cost, Node* prev_node) 
    : x_(x), y_(y), cost_(cost), prev_node_(prev_node)
{
}

std::string Node::asString() {
    return std::to_string(x_) + ", " + std::to_string(y_)
        + ", " + std::to_string(cost_);
}