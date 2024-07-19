#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("rmcs_navigation");

    using namespace std::chrono_literals;
    auto timer_ = node->create_wall_timer(1s, [&node] {
        RCLCPP_INFO(node->get_logger(), "Hello world");
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
}