#include "Node.h"


std::string Node::asString() {
    return std::to_string(x_) + ", " + std::to_string(y_)
        + ", " + std::to_string(cost_);
}