#include <pure_pursuit_controller.hpp>

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller"){
    // Initializing the Publisher
    vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Initializaing the Subscribers
    path_subscriber = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&PurePursuitController::process_path, this, std::placeholders::_1));
    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PurePursuitController::process_odom, this, std::placeholders::_1));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

void PurePursuitController::process_path(const nav_msgs::msg::Path::SharedPtr path_msg) {
    this->current_robot_x = 0.0;
}

geometry_msgs::msg::Pose::SharedPtr PurePursuitController::transform_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    geometry_msgs::msg::TransformStamped t;

    try {
        t = tf_buffer->lookupTransform("world", "odom", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", "world", "odom", ex.what());
        return nullptr;
    }

    geometry_msgs::msg::Pose transformed_pose;
    tf2::doTransform(odom_msg->pose.pose, transformed_pose, t);
    
    return std::make_shared<geometry_msgs::msg::Pose>(transformed_pose);
}

void PurePursuitController::process_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    auto transformed_pose = this->transform_odom(odom_msg);
    
    this->current_robot_x = transformed_pose->position.x;
    this->current_robot_y = transformed_pose->position.y;
    RCLCPP_INFO(this->get_logger(), "The current position of the robot is (%f, %f)", this->current_robot_x, this->current_robot_y);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}