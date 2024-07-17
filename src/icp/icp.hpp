#pragma once

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/node.hpp>

using PointT = pcl::PointXYZ;

class ICP final {
public:
    using PointT = pcl::PointXYZ;
    ICP();
    ~ICP();
    ICP(const ICP&) = delete;
    ICP& operator=(const ICP&) = delete;

    void register_map(const std::shared_ptr<pcl::PointCloud<PointT>>& map);
    void register_scan(const std::shared_ptr<pcl::PointCloud<PointT>>& scan);
    [[nodiscard]] bool calculate(const std::shared_ptr<pcl::PointCloud<PointT>>& output);

private:
    class Impl;
    Impl* impl_;
};
