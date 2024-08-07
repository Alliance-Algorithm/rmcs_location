#include "match.hpp"

#include <Eigen/Eigen>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

#include <fast_gicp/gicp/fast_gicp.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

static inline const auto node_options =
    rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);

class Registration::Impl : public rclcpp::Node {
public:
    using PointCloudT = pcl::PointCloud<PointT>;

    Impl()
        : Node("rmcs_location_gicp", node_options) {
        fast_gicp_engine_ = std::make_unique<fast_gicp::FastGICP<PointT, PointT>>();
        fast_gicp_engine_->setMaximumIterations(get_parameter_or<int>("gicp.maximum_iterations_detailed", 0));
        fast_gicp_engine_->setMaxCorrespondenceDistance(
            get_parameter_or<double>("gicp.distance_threshold", 0));
        fast_gicp_engine_->setTransformationEpsilon(
            get_parameter_or<double>("gicp.transformation_epsilon", 0));
        fast_gicp_engine_->setEuclideanFitnessEpsilon(
            get_parameter_or<double>("gicp.euclidean_fitness_epsilon", 0));
        fast_gicp_engine_->setNumThreads(get_parameter_or<int>("gicp.threads", 1));

        RCLCPP_INFO(get_logger(), "[gicp][maximum_iterations] %d", fast_gicp_engine_->getMaximumIterations());
        RCLCPP_INFO(
            get_logger(), "[gicp][distance_threshold] %f", fast_gicp_engine_->getMaxCorrespondenceDistance());
        RCLCPP_INFO(
            get_logger(), "[gicp][transformation_epsilon] %f", fast_gicp_engine_->getTransformationEpsilon());
        RCLCPP_INFO(
            get_logger(), "[gicp][euclidean_fitness_epsilon] %f",
            fast_gicp_engine_->getEuclideanFitnessEpsilon());
        RCLCPP_INFO(get_logger(), "[gicp][threads] %d", get_parameter_or<int>("gicp.threads", 1));

        outlier_removal_filter_ = std::make_unique<pcl::StatisticalOutlierRemoval<PointT>>();
        outlier_removal_filter_->setMeanK(get_parameter_or<int>("outlier_removal.mean_k", 0));
        outlier_removal_filter_->setStddevMulThresh(
            get_parameter_or<double>("outlier_removal.stddev_mul_thresh", 0.));

        RCLCPP_INFO(get_logger(), "[outlier_removal][mean_k] %d", outlier_removal_filter_->getMeanK());
        RCLCPP_INFO(
            get_logger(), "[outlier_removal][stddev_mul_thresh] %f",
            outlier_removal_filter_->getStddevMulThresh());

        voxel_grid_filter_ = std::make_unique<pcl::VoxelGrid<PointT>>();
        voxel_grid_filter_->setLeafSize(
            get_parameter_or<float>("voxel_grid.lx", 0), get_parameter_or<float>("voxel_grid.ly", 0),
            get_parameter_or<float>("voxel_grid.lz", 0));

        RCLCPP_INFO(
            get_logger(), "[voxel_grid][leaf_size] %f %f %f", voxel_grid_filter_->getLeafSize().x(),
            voxel_grid_filter_->getLeafSize().y(), voxel_grid_filter_->getLeafSize().z());

        aligned_ = false;
    }

    void register_map(const std::shared_ptr<pcl::PointCloud<PointT>>& map) {
        voxel_grid_filter_->setInputCloud(map);
        voxel_grid_filter_->filter(*map);
        outlier_removal_filter_->setInputCloud(map);
        outlier_removal_filter_->filter(*map);

        RCLCPP_INFO(get_logger(), "register map, size: %zu", map->size());
        fast_gicp_engine_->setInputTarget(map);

        aligned_ = false;
    }

    void register_scan(const std::shared_ptr<pcl::PointCloud<PointT>>& scan) {
        voxel_grid_filter_->setInputCloud(scan);
        voxel_grid_filter_->filter(*scan);

        RCLCPP_INFO(get_logger(), "register scan, size: %zu", scan->size());
        fast_gicp_engine_->setInputSource(scan);

        aligned_ = false;
    }

    inline void single_match(const std::shared_ptr<PointCloudT>& align) {
        fast_gicp_engine_->align(*align);

        aligned_        = true;
        fitness_score_  = fast_gicp_engine_->getFitnessScore(1.0);
        transformation_ = Eigen::Affine3f{fast_gicp_engine_->getFinalTransformation()};
    }

    inline void
        single_match(const std::shared_ptr<PointCloudT>& align, const Eigen::Affine3f& transformation) {
        fast_gicp_engine_->align(*align, transformation.matrix());

        aligned_        = true;
        fitness_score_  = fast_gicp_engine_->getFitnessScore(1.0);
        transformation_ = Eigen::Affine3f{fast_gicp_engine_->getFinalTransformation()};
    }

    void full_match(const std::shared_ptr<PointCloudT>& align) {
        // @question why not construct to identity matrix
        // auto no_translation = Eigen::Translation3f {};
        // auto no_rotation = Eigen::Quaternionf {};

        auto no_translation = Eigen::Translation3f::Identity();
        auto no_rotation    = Eigen::Quaternionf::Identity();

        full_match(align, {no_rotation * no_translation});
    }

    void full_match(const std::shared_ptr<PointCloudT>& align, const Eigen::Affine3f& transformation) {
        // rotate and get score, select the best angle
        double score_min = 1.0;
        int angle_best   = 0;

        // set maximum iterations for rough match
        fast_gicp_engine_->setMaximumIterations(get_parameter_or<int>("gicp.maximum_iterations_rough", 0));
        RCLCPP_INFO(get_logger(), "[maximum_iterations_rough] %d", fast_gicp_engine_->getMaximumIterations());

        static const auto scan_angle      = get_parameter_or<int>("gicp.scan_angle", 6);
        static const auto score_threshold = get_parameter_or<double>("gicp.score_threshold", 0.3);

        for (auto n = 1; (scan_angle * n / 2) < 181; n++) {

            const auto angle = static_cast<int>(scan_angle * static_cast<int>(n / 2) * std::pow(-1, n));

            auto radian   = static_cast<float>(static_cast<float>(angle) / 180 * std::numbers::pi);
            auto rotation = Eigen::AngleAxisf(radian, Eigen::Vector3f::UnitZ());
            auto guess    = (rotation * transformation).matrix();

            fast_gicp_engine_->align(*align, guess);
            auto score = fast_gicp_engine_->getFitnessScore(1.0);

            if (score < score_min) {
                score_min  = score;
                angle_best = angle;
            }

            RCLCPP_INFO(get_logger(), "[angle] %+4d [score] %-.5lf", angle, score);

            if (score < score_threshold) {
                RCLCPP_INFO(get_logger(), "[congratulate] score(%.5f) is enough, break", score);
                break;
            }
        }

        RCLCPP_INFO(get_logger(), "[angle_best] %-3d", angle_best);
        RCLCPP_INFO(get_logger(), "[score_best] %-3f", score_min);

        // set maximum iterations for detailed match
        fast_gicp_engine_->setMaximumIterations(get_parameter_or<int>("gicp.maximum_iterations_detailed", 0));
        RCLCPP_INFO(
            get_logger(), "[maximum_iterations_detailed] %d", fast_gicp_engine_->getMaximumIterations());

        // use the best rotation to align
        auto radian   = static_cast<float>(static_cast<float>(angle_best) / 180 * std::numbers::pi);
        auto rotation = Eigen::AngleAxisf(radian, Eigen::Vector3f::UnitZ());
        auto guess    = (rotation * transformation).matrix();

        fast_gicp_engine_->align(*align, guess);

        RCLCPP_INFO(
            get_logger(), "[score][%d times] %-3f", fast_gicp_engine_->getMaximumIterations(),
            fast_gicp_engine_->getFitnessScore(1.0));

        aligned_        = true;
        fitness_score_  = fast_gicp_engine_->getFitnessScore(1.0);
        transformation_ = Eigen::Affine3f{fast_gicp_engine_->getFinalTransformation()};
    }

    double fitness_score() const {
        assert(aligned_ == true);

        return fitness_score_;
    }

    Eigen::Affine3f transformation() const {
        assert(aligned_ == true);

        return transformation_;
    }

private:
    std::unique_ptr<fast_gicp::FastGICP<PointT, PointT>> fast_gicp_engine_;
    std::unique_ptr<pcl::StatisticalOutlierRemoval<PointT>> outlier_removal_filter_;
    std::unique_ptr<pcl::VoxelGrid<PointT>> voxel_grid_filter_;

    bool aligned_                   = false;
    double fitness_score_           = 0;
    Eigen::Affine3f transformation_ = Eigen::Affine3f::Identity();
};

void Registration::register_map(const std::shared_ptr<pcl::PointCloud<PointT>>& map) {
    impl_->register_map(map);
}
void Registration::register_scan(const std::shared_ptr<pcl::PointCloud<PointT>>& scan) {
    impl_->register_scan(scan);
}

void Registration::full_match(const std::shared_ptr<PointCloudT>& align) { impl_->full_match(align); }

void Registration::single_match(const std::shared_ptr<PointCloudT>& align) { impl_->single_match(align); }

void Registration::full_match(
    const std::shared_ptr<pcl::PointCloud<PointT>>& align, const Eigen::Affine3f& transformation) {
    impl_->full_match(align, transformation);
}

void Registration::single_match(
    const std::shared_ptr<pcl::PointCloud<PointT>>& align, const Eigen::Affine3f& transformation) {
    impl_->single_match(align, transformation);
}

double Registration::fitness_score() const { return impl_->fitness_score(); }

Eigen::Affine3f Registration::transformation() const { return impl_->transformation(); }

Registration::Registration() { impl_ = new Impl{}; }
Registration::~Registration() { delete impl_; }
