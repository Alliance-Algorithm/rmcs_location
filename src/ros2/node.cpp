#include "ros2/node.hpp"
#include "utility/convert.hpp"

#include <Eigen/Eigen>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace nav {

// clang-format off
constexpr auto slogan =
    "\n┌───────────────────────────────────────────────────────────────────┐"
    "\n│ welcome to use rmcs navigation, wish you a happy robomaster game! │"
    "\n└───────────────────────────────────────────────────────────────────┘";
// clang-format on

static const auto option = rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);

Node::Node()
    : rclcpp::Node("rmcs_location", option) {
    using namespace std::chrono_literals;

    RCLCPP_INFO(get_logger(), slogan);

    // ros2 interface create
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rmcs_location/pose", 10);

    slam_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/rmcs_slam/pose", rclcpp::SensorDataQoS{},
        [this](const std::unique_ptr<geometry_msgs::msg::PoseStamped>& msg) {
            slam_pose_subscription_callback(msg);
        });
    slam_map_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rmcs_slam/laser_map", rclcpp::SensorDataQoS{},
        [this](const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
            slam_map_subscription_callback(msg);
        });

    static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    process_timer_ = create_wall_timer(100ms, [this] { process_timer_callback(); });

    slam_reset_trigger_ = create_client<std_srvs::srv::Trigger>("/rmcs_slam/reset");

    // gicp and pointcloud create
    map_standard_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    map_initial_  = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    gicp_         = std::make_shared<Registration>();

    // load some parameter
    initialize_pose_       = get_parameter_or("initialize_pose", false);
    localization_when_lost = get_parameter_or("localization_when_lost", false);
    lidar_quaternion_      = Eigen::Quaterniond{
        Eigen::AngleAxisd{
                          get_parameter_or<double>("lidar_to_odom.AngleAxis.x", 0.0), Eigen::Vector3d::UnitX()}
        * Eigen::
            AngleAxisd{get_parameter_or<double>("lidar_to_odom.AngleAxis.y", 0.0), Eigen::Vector3d::UnitY()}
        * Eigen::AngleAxisd{
                          get_parameter_or<double>("lidar_to_odom.AngleAxis.z", 0.0), Eigen::Vector3d::UnitZ()}
    };
    lidar_translation_ = Eigen::Translation3d{
        get_parameter_or<double>("lidar_to_odom.Translation.x", 0.0),
        get_parameter_or<double>("lidar_to_odom.Translation.y", 0.0),
        get_parameter_or<double>("lidar_to_odom.Translation.z", 0.0)};

    const auto path = get_parameter_or<std::string>("path_map", "map is not found");
    if (-1 == pcl::io::loadPCDFile(path, *map_standard_)) {
        RCLCPP_ERROR(get_logger(), "Couldn't read file: %s\n", path.c_str());
        initialize_pose_ = false;
    } else {
        RCLCPP_INFO(get_logger(), "read standard map file at: %s", path.c_str());
    }

    // handle client
    while (!slam_reset_trigger_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "interrupted while waiting for the service");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    if (initialize_pose_)
        get_initial_map_ = true;
}

void Node::slam_pose_subscription_callback(const std::unique_ptr<geometry_msgs::msg::PoseStamped>& msg) {
    auto stamp = geometry_msgs::msg::PoseStamped{};

    stamp.header.stamp    = msg->header.stamp;
    stamp.header.frame_id = "world";

    auto translation = Eigen::Vector3d{};
    auto rotation    = Eigen::Quaterniond{};
    utility::set_translation(translation, msg->pose.position);
    utility::set_quaternion(rotation, msg->pose.orientation);

    translation = lidar_quaternion_ * lidar_translation_ * initialization_transformation_ * translation;
    rotation    = lidar_quaternion_ * initialization_transformation_.rotation() * rotation;

    utility::set_translation(stamp.pose.position, translation);
    utility::set_quaternion(stamp.pose.orientation, rotation);

    pose_publisher_->publish(stamp);

    if (localization_when_lost)
        if (std::abs(translation.z()) > 3 || std::abs(translation.y()) > 10 || std::abs(translation.x()) > 20)
            status_ = Status::LOST;
}

void Node::slam_map_subscription_callback(const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
    if (!get_initial_map_)
        return;

    map_initial_->clear();
    pcl::fromROSMsg(*msg, *map_initial_);

    get_initial_map_ = false;
    RCLCPP_INFO(get_logger(), "get initial map, size: %zu", map_initial_->size());
}

void Node::process_timer_callback() {
    switch (status_) {
    // @brief wait the begin of livox and slam
    case Status::WAIT: {
        if (!initialize_pose_) {
            RCLCPP_INFO(get_logger(), "running without localization");
            publish_static_transform();
            status_ = Status::RUNNING;
            return;
        }

        if (!get_initial_map_) {
            status_ = Status::PREPARED;
            RCLCPP_INFO(get_logger(), "slam is running successfully, prepared for pointcloud registration");
            return;
        }

        return;
    }
    // @brief collect enough lidar frame to localize
    case Status::PREPARED: {
        static const auto size = get_parameter_or<int>("registration_size", 2000);
        if (map_initial_ == nullptr || map_initial_->size() < size) {
            // initial map is useless, retry to get another map
            get_initial_map_ = true;
            return;
        } else {
            // initial map is enough
            RCLCPP_INFO(get_logger(), "finish collecting pointcloud");
            status_ = Status::LOCALIZATION;
            return;
        }
        return;
    }
    // @brief localization
    case Status::LOCALIZATION: {
        RCLCPP_INFO(get_logger(), "gicp match start");
        initialize_pose();
        RCLCPP_INFO(get_logger(), "gicp match end");
        status_ = Status::RUNNING;
        return;
    }
    // @brief runtime after localization
    case Status::RUNNING: {
        return;
    }
    // @brief localize again after losing the position
    case Status::LOST: {
        slam_reset_trigger_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

        map_initial_->clear();
        get_initial_map_ = true;
        status_          = Status::WAIT;
        return;
    }
    }
}

void Node::initialize_pose() {
    if (!initialize_pose_) {
        RCLCPP_INFO(get_logger(), "running without localization");
        return;
    }

    gicp_->register_map(map_standard_);
    gicp_->register_scan(map_initial_);

    auto align = std::make_shared<Registration::PointCloudT>();
    gicp_->full_match(align);

    initialization_transformation_ = gicp_->transformation().cast<double>();

    publish_static_transform();
}

void Node::publish_static_transform() {
    auto transform_stamp = geometry_msgs::msg::TransformStamped();

    auto transform = initialization_transformation_.inverse();

    utility::set_quaternion(
        transform_stamp.transform.rotation, Eigen::Quaterniond{transform.rotation() * lidar_quaternion_});

    utility::set_translation(
        transform_stamp.transform.translation, Eigen::Vector3d{transform * lidar_translation_.translation()});

    transform_stamp.header.stamp = get_clock()->now();

    // @note frame_id exists before child_frame_id
    transform_stamp.header.frame_id = "lidar_init";
    transform_stamp.child_frame_id  = "world";

    static_transform_broadcaster_->sendTransform(transform_stamp);
}
} // namespace nav
