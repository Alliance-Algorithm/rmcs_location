#include <cstdio>
#include <memory>
#include <string>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/executors.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "gicp/gicp.hpp"

void publish_static_transformation(rclcpp::Node& node, tf2_ros::StaticTransformBroadcaster& broadcaster)
{
    auto transform_stamp = geometry_msgs::msg::TransformStamped();

    auto quaternion = Eigen::Quaterniond {
        Eigen::AngleAxisd { std::numbers::pi, Eigen::Vector3d::UnitY() }
    };

    transform_stamp.transform.rotation.w = quaternion.w();
    transform_stamp.transform.rotation.x = quaternion.x();
    transform_stamp.transform.rotation.y = quaternion.y();
    transform_stamp.transform.rotation.z = quaternion.z();

    transform_stamp.transform.translation.z = 0.6;

    transform_stamp.header.stamp = node.get_clock()->now();
    transform_stamp.header.frame_id = "world";
    transform_stamp.child_frame_id = "lidar_init";
    broadcaster.sendTransform(transform_stamp);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::signal(SIGINT, [](int signal) {
        std::cout << "Bye!" << std::endl;
        exit(signal);
    });

    auto node = std::make_shared<rclcpp::Node>(
        "rmcs_navigation_test", rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true));

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> scan_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> trans_publisher_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    map_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_navigation_test/map", 10);
    scan_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_navigation_test/scan", 10);
    trans_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_navigation_test/trans", 10);
    pose_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/rmcs_navigation_test/pose", 10);
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    publish_static_transformation(*node, *broadcaster_);

    auto map = std::make_shared<pcl::PointCloud<GICP::PointT>>();
    auto scan = std::make_shared<pcl::PointCloud<GICP::PointT>>();

    auto path1 = node->get_parameter_or<std::string>("path_map", "/temp");
    auto path2 = node->get_parameter_or<std::string>("path_scan", "/temp");

    if (pcl::io::loadPCDFile<GICP::PointT>(path1, *map) == -1) {
        PCL_ERROR("Couldn't read file: %s\n", path1.c_str());
        return (-1);
    }

    if (pcl::io::loadPCDFile<GICP::PointT>(path2, *scan) == -1) {
        PCL_ERROR("Couldn't read file: %s\n", path2.c_str());
        return (-1);
    }

    // start the timer
    auto time_start = node->get_clock()->now();

    auto gicp = GICP {};
    gicp.register_map(map);
    gicp.register_scan(scan);

    auto align = std::make_shared<pcl::PointCloud<GICP::PointT>>();
    gicp.full_match(align);

    auto time_end = node->get_clock()->now();

    auto time_cost = time_end - time_start;

    RCLCPP_INFO(node->get_logger(), "[time] %lf", time_cost.seconds());

    std::cout << "[fitness_score]" << std::endl;
    std::cout << gicp.fitness_score() << std::endl;

    const auto& transformation = gicp.transformation();
    auto rotation = transformation.rotation().eulerAngles(0, 1, 2);
    auto quaternion = Eigen::Quaternionf { transformation.rotation() };
    auto translation = transformation.translation();

    std::cout << "[rotation]" << std::endl;
    std::cout << rotation << std::endl;
    std::cout << "[translation]" << std::endl;
    std::cout << translation << std::endl;

    auto pose = geometry_msgs::msg::PoseStamped {};
    pose.header.frame_id = "lidar_init";
    pose.header.stamp = node->get_clock()->now();
    pose.pose.position.x = translation.x();
    pose.pose.position.y = translation.y();
    pose.pose.position.z = translation.z();
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();

    pose_publisher_->publish(pose);

    if (false) {
        auto pcd = GICP::PointCloudT {};

        pcd += *map;
        pcd += *align;

        pcl::io::savePCDFileASCII("/workspaces/sentry/ignore/develop_ws/standard.pcd", pcd);
    }

    using namespace std::chrono_literals;
    timer_ = node->create_wall_timer(
        1s, [&map_publisher_, &scan_publisher_, &trans_publisher_, &map, &scan, &align, &node, &pose, &pose_publisher_] {
            static auto map_publish = sensor_msgs::msg::PointCloud2();
            static auto scan_publish = sensor_msgs::msg::PointCloud2();
            static auto transformed = sensor_msgs::msg::PointCloud2();

            pcl::toROSMsg(*map, map_publish);
            map_publish.header.frame_id = "lidar_init";
            map_publish.header.stamp = node->get_clock()->now();

            pcl::toROSMsg(*scan, scan_publish);
            scan_publish.header.frame_id = "lidar_init";
            scan_publish.header.stamp = node->get_clock()->now();

            pcl::toROSMsg(*align, transformed);
            transformed.header.frame_id = "lidar_init";
            transformed.header.stamp = node->get_clock()->now();

            map_publisher_->publish(map_publish);
            scan_publisher_->publish(scan_publish);
            trans_publisher_->publish(transformed);
        });

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}