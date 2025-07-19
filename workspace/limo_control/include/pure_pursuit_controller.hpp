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
    const float lookahead_dist = 0.3;
    const float max_speed = 0.1;
    const float heading_const = 4;
    const float kp_ = 0.8;
    const float ki_ = 0.1;
    const float kd_ = 0.3;

    bool init_ang_adj = false;
    bool final_adj = false;
    int base_point = 0;
    float angular_error = 0.0;
    float linear_error = 0.0;
    float path_yaw = 0.0;

    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::PoseStamped target_pose;
    geometry_msgs::msg::Twist output_cmd_vel;
    nav_msgs::msg::Path robot_path;
    geometry_msgs::msg::TransformStamped transform;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    PIDController pid_controller;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    void process_path(const nav_msgs::msg::Path::SharedPtr path_msg);
    void process_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    
    void update_current_pose(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void find_target_pose();
    void update_errors();
    void update_control();
    void pure_pursuit();
};