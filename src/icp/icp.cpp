#include "icp.hpp"

#include <Eigen/Eigen>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

std::tuple<Eigen::Matrix4f, std::shared_ptr<pcl::PointCloud<PointT>>>
calculate_transform(
    const std::shared_ptr<pcl::PointCloud<PointT>>& scan,
    const std::shared_ptr<pcl::PointCloud<PointT>>& map)
{
    static auto node = rclcpp::Node("icp", rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true));

    // downsample filter
    static auto filter = pcl::VoxelGrid<PointT> {};
    filter.setLeafSize(0.1f, 0.1f, 0.1f);

    filter.setInputCloud(map);
    filter.filter(*map);

    filter.setInputCloud(scan);
    filter.filter(*scan);

    // remove outlier
    static auto removal = pcl::StatisticalOutlierRemoval<PointT> {};
    removal.setMeanK(50);
    removal.setStddevMulThresh(0.3);
    removal.setInputCloud(map);
    removal.filter(*map);

    // icp

    static auto cloud_align_icp
        = std::make_shared<pcl::PointCloud<PointT>>();

    static auto icp = pcl::IterativeClosestPoint<PointT, PointT> {};
    icp.setInputTarget(map);
    icp.setInputSource(scan);
    icp.setMaximumIterations(50);
    icp.setMaxCorrespondenceDistance(5);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    for (auto angle = 0; angle < 361; angle += 10) {
        auto rotation = Eigen::AngleAxisf(static_cast<float>(static_cast<float>(angle) / 180 * std::numbers::pi), Eigen::Vector3f::UnitZ());
        auto translation = Eigen::Translation3f {};
        auto affine = (rotation * translation).matrix();

        icp.align(*cloud_align_icp, affine);
        RCLCPP_INFO(rclcpp::get_logger("icp"), "[angle] %-3d [score] %-.5lf", angle, icp.getFitnessScore(1.0));
    }

    icp.align(*cloud_align_icp);
    auto transformation = icp.getFinalTransformation();

    return { transformation, cloud_align_icp };
}