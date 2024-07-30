#include <rclcpp/executors.hpp>

#include "ros2/node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<nav::Node>());

    rclcpp::shutdown();
}