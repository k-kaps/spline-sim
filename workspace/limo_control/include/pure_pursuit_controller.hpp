#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class PurePursuitController : public rclcpp::Node {
public:

    PurePursuitController();

private:

    void process_path(const nav_msgs::msg::Path::SharedPtr path_msg);
    void process_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    geometry_msgs::msg::Pose::SharedPtr transform_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber;

    float current_robot_x = 0.0;
    float current_robot_y = 0.0;

    const float lookahead_dist = 0.4; // in meters

    std::string target_frame = "world";

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};