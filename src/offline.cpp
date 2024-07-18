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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/executors.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "gicp/gicp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::signal(SIGINT, [](int signal) {
        std::cout << "Bye!" << std::endl;
        exit(signal);
    });

    auto node = std::make_shared<rclcpp::Node>(
        "rmcs_navigation", rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true));

    auto path1 = node->get_parameter_or<std::string>("path_map", "/temp");
    auto path2 = node->get_parameter_or<std::string>("path_scan", "/temp");

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> scan_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> trans_publisher_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    map_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/map", 10);
    scan_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/scan", 10);
    trans_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/trans", 10);

    auto map = std::make_shared<pcl::PointCloud<GICP::PointT>>();
    auto scan = std::make_shared<pcl::PointCloud<GICP::PointT>>();

    if (pcl::io::loadPCDFile<GICP::PointT>(path1, *map) == -1) {
        PCL_ERROR("Couldn't read file: %s\n", path1.c_str());
        return (-1);
    }

    if (pcl::io::loadPCDFile<GICP::PointT>(path2, *scan) == -1) {
        PCL_ERROR("Couldn't read file: %s\n", path2.c_str());
        return (-1);
    }

    auto time_start = node->get_clock()->now();

    auto gicp = GICP {};
    gicp.register_map(map);
    gicp.register_scan(scan);

    auto cloud_align = std::make_shared<pcl::PointCloud<GICP::PointT>>();
    gicp.full_match(cloud_align);

    auto time_end = node->get_clock()->now();

    auto time_cost = time_end - time_start;

    RCLCPP_INFO(node->get_logger(), "[time] %lf", time_cost.seconds());

    using namespace std::chrono_literals;
    timer_ = node->create_wall_timer(
        1s, [&map_publisher_, &scan_publisher_, &trans_publisher_, &map, &scan, &cloud_align, &node] {
            static auto map_publish = sensor_msgs::msg::PointCloud2();
            static auto scan_publish = sensor_msgs::msg::PointCloud2();
            static auto transformed = sensor_msgs::msg::PointCloud2();

            pcl::toROSMsg(*map, map_publish);
            map_publish.header.frame_id = "test_link";
            map_publish.header.stamp = node->get_clock()->now();

            pcl::toROSMsg(*scan, scan_publish);
            scan_publish.header.frame_id = "test_link";
            scan_publish.header.stamp = node->get_clock()->now();

            pcl::toROSMsg(*cloud_align, transformed);
            transformed.header.frame_id = "test_link";
            transformed.header.stamp = node->get_clock()->now();

            map_publisher_->publish(map_publish);
            scan_publisher_->publish(scan_publish);
            trans_publisher_->publish(transformed);
        });

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}