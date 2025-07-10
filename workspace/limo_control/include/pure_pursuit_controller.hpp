#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.hpp"

#include <pid_controller.hpp>

class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController();

private:
    // Constants
    const float ld = 0.3; // in meters
    const float K_p = 0.4;

    void process_path(const nav_msgs::msg::Path::SharedPtr path_msg);
    void process_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void transform_robot_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    bool store_target_pose();
    void compute_cmd_vel();
    void compute_errors();
    void normalize_angular_error();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ld_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber;
    
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::PoseStamped target_pose;
    nav_msgs::msg::Path robot_path;

    std::string target_frame = "world";

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    geometry_msgs::msg::TransformStamped transform;

    float angular_error = 0.0;
    float linear_error = 0.0;
    int start_pt = 0;

    PIDController pid_controller;
};