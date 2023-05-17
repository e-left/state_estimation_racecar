#include <iostream>
#include "EstimationNode.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto StateEstimationNode = std::make_shared<EstimationNode>();
    rclcpp::spin(StateEstimationNode);
    rclcpp::shutdown();
    return 0;
}
