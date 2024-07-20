#pragma once

#include <Eigen/Eigen>

#include <rclcpp/node.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace nav {

constexpr auto slogan
    = "\n┌───────────────────────────────────────────────────────────────────┐"
      "\n│ welcome to use rmcs navigation, wish you a happy robomaster game! │"
      "\n└───────────────────────────────────────────────────────────────────┘";

static const auto option = rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true);

class Node : public rclcpp::Node {
public:
    explicit Node()
        : rclcpp::Node("rmcs_navigation", option)
    {
        RCLCPP_INFO(get_logger(), slogan);

        static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_static_transform();
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;

    void publish_static_transform()
    {
        auto transform_stamp = geometry_msgs::msg::TransformStamped();

        auto quaternion = Eigen::Quaterniond {
            Eigen::AngleAxisd { std::numbers::pi, Eigen::Vector3d::UnitY() }
        };

        transform_stamp.transform.rotation.w = quaternion.w();
        transform_stamp.transform.rotation.x = quaternion.x();
        transform_stamp.transform.rotation.y = quaternion.y();
        transform_stamp.transform.rotation.z = quaternion.z();

        transform_stamp.transform.translation.x = 0;
        transform_stamp.transform.translation.y = 0;
        transform_stamp.transform.translation.z = 0.6;

        transform_stamp.header.stamp = get_clock()->now();

        // @note frame_id exists before child_frame_id
        transform_stamp.header.frame_id = "lidar_init";
        transform_stamp.child_frame_id = "world";

        static_transform_broadcaster_->sendTransform(transform_stamp);
    }
};
}