#include "ros2/node.hpp"
#include "gicp/gicp.hpp"
#include "utility/convert.hpp"

#include <Eigen/Eigen>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace nav {

constexpr auto slogan
    = "\n┌───────────────────────────────────────────────────────────────────┐"
      "\n│ welcome to use rmcs navigation, wish you a happy robomaster game! │"
      "\n└───────────────────────────────────────────────────────────────────┘";

static const auto option = rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true);

Node::Node()
    : rclcpp::Node("rmcs_navigation", option)
{
    using namespace std::chrono_literals;

    RCLCPP_INFO(get_logger(), slogan);

    // ros2 interface create
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rmcs_navigation/pose", rclcpp::SensorDataQoS {});

    livox_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", rclcpp::SensorDataQoS {},
        [this](const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) { livox_subscription_callback(msg); });

    slam_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>("/rmcs_slam/pose", rclcpp::SensorDataQoS {},
        [this](const std::unique_ptr<geometry_msgs::msg::PoseStamped>& msg) { slam_pose_subscription_callback(msg); });

    slam_map_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>("/rmcs_slam/map", rclcpp::SensorDataQoS {},
        [this](const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {});

    static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    process_timer_ = create_wall_timer(100ms, [this] { process_timer_callback(); });

    slam_reset_trigger_ = create_client<std_srvs::srv::Trigger>("/rmcs_slam/reset");

    // gicp and pointcloud create
    map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    scan_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    gicp_ = std::make_shared<GICP>();

    const auto path = get_parameter_or<std::string>("path_map", "map is not found");
    if (-1 == pcl::io::loadPCDFile(path, *map_)) {
        RCLCPP_ERROR(get_logger(), "Couldn't read file: %s\n", path.c_str());
        rclcpp::shutdown();
    }

    // load some parameter
    quaternion_ = Eigen::Quaterniond {
        Eigen::AngleAxisd { get_parameter_or<double>("map_to_world.AngleAxis.x", 0.0), Eigen::Vector3d::UnitX() }
        * Eigen::AngleAxisd { get_parameter_or<double>("map_to_world.AngleAxis.y", 0.0), Eigen::Vector3d::UnitY() }
        * Eigen::AngleAxisd { get_parameter_or<double>("map_to_world.AngleAxis.z", 0.0), Eigen::Vector3d::UnitZ() }
    };
    translation_ = Eigen::Translation3d {
        get_parameter_or<double>("map_to_world.Translation.x", 0.0),
        get_parameter_or<double>("map_to_world.Translation.y", 0.0),
        get_parameter_or<double>("map_to_world.Translation.z", 0.0)
    };

    publish_static_transform();

    // handle client
    while (!slam_reset_trigger_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
}

void Node::livox_subscription_callback(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg)
{
    lidar_ready_ = true;

    if (get_lidar_frame_) {
        scan_->clear();

        for (const auto& point : msg->points)
            scan_->points.emplace_back(point.x, point.y, point.z);

        get_lidar_frame_ = false;
    }
}

void Node::slam_pose_subscription_callback(const std::unique_ptr<geometry_msgs::msg::PoseStamped>& msg)
{
    auto stamp = geometry_msgs::msg::PoseStamped {};

    stamp.header.stamp = msg->header.stamp;
    stamp.header.frame_id = "world";

    auto translation = Eigen::Vector3d {};
    auto rotation = Eigen::Quaterniond {};
    utility::set_translation(translation, msg->pose.position);
    utility::set_quaternion(rotation, msg->pose.orientation);

    translation = initialization_transformation_ * translation;
    rotation = initialization_transformation_.rotation() * rotation;

    translation = quaternion_ * translation_ * translation;
    rotation = quaternion_ * rotation;

    utility::set_translation(stamp.pose.position, translation);
    utility::set_quaternion(stamp.pose.orientation, rotation);

    pose_publisher_->publish(stamp);
}

void Node::process_timer_callback()
{
    switch (status_) {

    // @brief wait the begin of livox and slam
    case Status::WAIT: {

        if (lidar_ready_) {
            status_ = Status::PREPARED;
            RCLCPP_INFO(get_logger(), "lidar is ready, collect pointcloud now");
        }

        return;
    }

    // @brief collect enough lidar frame to localize
    case Status::PREPARED: {

        if (scan_ == nullptr || scan_->size() < 1000) {
            status_ = Status::WAIT;
            get_lidar_frame_ = true;
        } else {
            RCLCPP_INFO(get_logger(), "finish collecting pointcloud");
            status_ = Status::LOCALIZATION;
        }

        return;
    }

    // @brief localization
    case Status::LOCALIZATION: {
        RCLCPP_INFO(get_logger(), "gicp match start");
        initialize_pose();
        status_ = Status::RUNNING;

        RCLCPP_INFO(get_logger(), "gicp match end");
        return;
    }

    // @brief runtime after localization
    case Status::RUNNING: {
        return;
    }

    // @brief localize again after losing the position
    case Status::LOST: {
        return;
    }
    }
}

void Node::initialize_pose()
{
    if (!get_parameter_or<bool>("initialize_pose", false)) {
        RCLCPP_INFO(get_logger(), "running without localization");
        return;
    }

    slam_reset_trigger_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    publish_static_transform();
}

void Node::publish_static_transform()
{
    auto transform_stamp = geometry_msgs::msg::TransformStamped();

    utility::set_quaternion(transform_stamp.transform.rotation, Eigen::Quaterniond { initialization_transformation_.rotation() * quaternion_ });
    utility::set_translation(transform_stamp.transform.translation, Eigen::Vector3d { initialization_transformation_ * translation_.translation() });

    transform_stamp.header.stamp = get_clock()->now();

    // @note frame_id exists before child_frame_id
    transform_stamp.header.frame_id = "lidar_init";
    transform_stamp.child_frame_id = "world";

    static_transform_broadcaster_->sendTransform(transform_stamp);
}
}
