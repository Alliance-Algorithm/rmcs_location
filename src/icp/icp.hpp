#pragma once

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZ;

std::tuple<Eigen::Matrix4f, std::shared_ptr<pcl::PointCloud<PointT>>>
calculate_transform(
    const std::shared_ptr<pcl::PointCloud<PointT>>& scan,
    const std::shared_ptr<pcl::PointCloud<PointT>>& map);