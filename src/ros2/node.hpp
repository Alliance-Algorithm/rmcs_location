#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/node.hpp>

#include "gicp/gicp.hpp"

namespace nav {

class Node : public rclcpp::Node {
public:
    explicit Node();

private:
    enum class Status {
        WAIT = 0,
        PREPARED,
        LOCALIZATION,
        RUNNING,
        LOST,
    };

private:
    // @note ros2 interface
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_publisher_;
    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> livox_subscription_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> slam_pose_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> slam_map_subscription_;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> slam_reset_trigger_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
    std::shared_ptr<rclcpp::TimerBase> process_timer_;

    // @note gicp
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> scan_;

    std::shared_ptr<GICP> gicp_;

    // @note rotation from map link "lidar_init" to world link "world"
    Eigen::Quaterniond quaternion_ { Eigen::Quaterniond::Identity() };

    // @note translation from map link "lidar_init" to world link "world"
    Eigen::Translation3d translation_ { Eigen::Translation3d::Identity() };

    // @note the init pose in map
    Eigen::Affine3d initialization_transformation_ { Eigen::Affine3d::Identity() };

    Status status_ { Status::WAIT };

    bool lidar_ready_ { false };
    bool get_lidar_frame_ { false };

private:
    // @brief receive pointcloud from livox lidar, about 20w points per second
    // @brief the frame is used to localize
    void livox_subscription_callback(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg);

    void slam_pose_subscription_callback(const std::unique_ptr<geometry_msgs::msg::PoseStamped>& msg);

    void slam_map_subscription_callback(const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg);

    // @brief main process
    void process_timer_callback();

    // @brief localization
    void initialize_pose();

    // @brief link transform: lidar_init -> world
    void publish_static_transform();
};
}