#include "icp.hpp"

#include <Eigen/Eigen>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

static inline auto node_options = rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true);

class ICP::Impl : public rclcpp::Node {
public:
    using PointCloudT = pcl::PointCloud<PointT>;

    Impl()
        : Node("rmcs_navigation_icp", node_options)
    {
        icp_engine_ = std::make_unique<pcl::IterativeClosestPoint<PointT, PointT>>();
        icp_engine_->setMaximumIterations(get_parameter_or<int>("icp.maximum_iterations_detailed", 0));
        icp_engine_->setMaxCorrespondenceDistance(get_parameter_or<double>("icp.distance_threshold", 0));
        icp_engine_->setTransformationEpsilon(get_parameter_or<double>("icp.transformation_epsilon", 0));
        icp_engine_->setEuclideanFitnessEpsilon(get_parameter_or<double>("icp.euclidean_fitness_epsilon", 0));

        RCLCPP_INFO(get_logger(), "[icp][maximum_iterations] %d", icp_engine_->getMaximumIterations());
        RCLCPP_INFO(get_logger(), "[icp][distance_threshold] %f", icp_engine_->getMaxCorrespondenceDistance());
        RCLCPP_INFO(get_logger(), "[icp][transformation_epsilon] %f", icp_engine_->getTransformationEpsilon());
        RCLCPP_INFO(get_logger(), "[icp][euclidean_fitness_epsilon] %f", icp_engine_->getEuclideanFitnessEpsilon());

        outlier_removal_filter_ = std::make_unique<pcl::StatisticalOutlierRemoval<PointT>>();
        outlier_removal_filter_->setMeanK(get_parameter_or<int>("outlier_removal.mean_k", 0));
        outlier_removal_filter_->setStddevMulThresh(get_parameter_or<double>("outlier_removal.stddev_mul_thresh", 0.));

        RCLCPP_INFO(get_logger(), "[outlier_removal][mean_k] %d", outlier_removal_filter_->getMeanK());
        RCLCPP_INFO(get_logger(), "[outlier_removal][stddev_mul_thresh] %f", outlier_removal_filter_->getStddevMulThresh());

        voxel_grid_filter_ = std::make_unique<pcl::VoxelGrid<PointT>>();
        voxel_grid_filter_->setLeafSize(
            get_parameter_or<float>("voxel_grid.lx", 0),
            get_parameter_or<float>("voxel_grid.ly", 0),
            get_parameter_or<float>("voxel_grid.lz", 0));

        RCLCPP_INFO(get_logger(), "[voxel_grid][leaf_size] %f %f %f",
            voxel_grid_filter_->getLeafSize().x(),
            voxel_grid_filter_->getLeafSize().y(),
            voxel_grid_filter_->getLeafSize().z());
    }

    void register_map(const std::shared_ptr<pcl::PointCloud<PointT>>& map)
    {
        voxel_grid_filter_->setInputCloud(map);
        voxel_grid_filter_->filter(*map);
        outlier_removal_filter_->setInputCloud(map);
        outlier_removal_filter_->filter(*map);

        RCLCPP_INFO(get_logger(), "register map, size: %zu", map->size());
        icp_engine_->setInputTarget(map);
    }

    void register_scan(const std::shared_ptr<pcl::PointCloud<PointT>>& scan)
    {
        voxel_grid_filter_->setInputCloud(scan);
        voxel_grid_filter_->filter(*scan);

        RCLCPP_INFO(get_logger(), "register scan, size: %zu", scan->size());
        icp_engine_->setInputSource(scan);
    }

    bool align(const std::shared_ptr<PointCloudT>& align)
    {
        icp_engine_->align(*align);
        return icp_engine_->hasConverged();
    }

    bool match(const std::shared_ptr<PointCloudT>& align)
    {
        auto no_translation = Eigen::Translation3f {};
        auto no_rotation = Eigen::Quaternionf {};
        return match(align, { no_rotation * no_translation });
    }

    bool match(const std::shared_ptr<PointCloudT>& align, const Eigen::Affine3f& transformation)
    {
        // rotate and get score, select the best angle
        double score_min = 1.0;
        int angle_best = 0;

        icp_engine_->setMaximumIterations(get_parameter_or<int>("icp.maximum_iterations_rough", 0));
        for (auto angle = 0; angle < 351; angle += 10) {
            auto radian = static_cast<float>(static_cast<float>(angle) / 180 * std::numbers::pi);
            auto rotation = Eigen::AngleAxisf(radian, Eigen::Vector3f::UnitZ());
            auto guess = (rotation * transformation).matrix();

            icp_engine_->align(*align, guess);
            auto score = icp_engine_->getFitnessScore(1.0);

            if (score < score_min) {
                score_min = score;
                angle_best = angle;
            }

            RCLCPP_INFO(rclcpp::get_logger("icp"), "[angle] %-3d [score] %-.5lf", angle, score);

            if (score_min < 0.03) {
                RCLCPP_INFO(rclcpp::get_logger("icp"), "score is enough, break");
                break;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("icp"), "[angle_best] %-3d", angle_best);
        RCLCPP_INFO(rclcpp::get_logger("icp"), "[score_best] %-3f", score_min);

        icp_engine_->setMaximumIterations(get_parameter_or<int>("icp.maximum_iterations_detailed", 0));

        // use the best rotation to align
        auto radian = static_cast<float>(static_cast<float>(angle_best) / 180 * std::numbers::pi);
        auto rotation = Eigen::AngleAxisf(radian, Eigen::Vector3f::UnitZ());
        auto guess = (rotation * transformation).matrix();

        icp_engine_->align(*align, guess);

        RCLCPP_INFO(rclcpp::get_logger("icp"), "[score][%d times] %-3f", icp_engine_->getMaximumIterations(), icp_engine_->getFitnessScore(1.0));

        return true;
    }

private:
    std::unique_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp_engine_;
    std::unique_ptr<pcl::StatisticalOutlierRemoval<PointT>> outlier_removal_filter_;
    std::unique_ptr<pcl::VoxelGrid<PointT>> voxel_grid_filter_;
};

void ICP::register_map(const std::shared_ptr<pcl::PointCloud<PointT>>& map) { impl_->register_map(map); }
void ICP::register_scan(const std::shared_ptr<pcl::PointCloud<PointT>>& scan) { impl_->register_scan(scan); }

[[nodiscard]] bool ICP::calculate(const std::shared_ptr<pcl::PointCloud<PointT>>& output)
{
    return impl_->match(output);
}

ICP::ICP() { impl_ = new Impl {}; }
ICP::~ICP() { delete impl_; }
