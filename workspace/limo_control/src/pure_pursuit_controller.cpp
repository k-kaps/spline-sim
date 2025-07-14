#include <pure_pursuit_controller.hpp>

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller") {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/lookahead_pose", 10);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&PurePursuitController::process_path, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PurePursuitController::process_odom, this, std::placeholders::_1));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    pid_controller = PIDController(kp_, ki_, kd_);
}

void PurePursuitController::process_path(const nav_msgs::msg::Path::SharedPtr path_msg) {
    robot_path = *path_msg;
}

void PurePursuitController::process_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    // transform the odom message to world coordinates
    update_current_pose(odom_msg);
    RCLCPP_INFO(this->get_logger(), "Current Pose (x: %f, y: %f), (z: %f)", current_pose.position.x, current_pose.position.y, current_pose.orientation.z);
    
    find_target_pose();
    RCLCPP_INFO(this->get_logger(), "Target Pose (x: %f, y: %f), (z: %f)", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.orientation.z);
    lookahead_pub_->publish(target_pose);

    update_errors();
    RCLCPP_INFO(this->get_logger(), "Linear Error: %f, Angular Error: %f", linear_error, angular_error);

    update_control();
}

void PurePursuitController::update_current_pose(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    try {
        transform = tf_buffer->lookupTransform("world", "odom", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", "world", "odom", ex.what());
        return;
    }
    tf2::doTransform(odom_msg->pose.pose, current_pose, transform);
}

void PurePursuitController::find_target_pose() {
    for (int i = start_pt; i < robot_path.poses.size(); i++) {
        double dist_x = robot_path.poses[i].pose.position.x - current_pose.position.x;
        double dist_y = robot_path.poses[i].pose.position.y - current_pose.position.y;
        double dist = std::sqrt((dist_x * dist_x) + (dist_y * dist_y));
        if (dist > lookahead_dist) {
            target_pose = robot_path.poses[i];
            prev_start_pt = start_pt;
            start_pt  = i;
            return;
        }
    }
    if (!robot_path.poses.empty()) {
        target_pose = robot_path.poses.back();
        return;
    }
}

void PurePursuitController::update_errors() {
    double dist_x = target_pose.pose.position.x - current_pose.position.x;
    double dist_y = target_pose.pose.position.y - current_pose.position.y;
    double current_yaw = tf2::getYaw(current_pose.orientation);
    
    path_yaw = std::atan2(dist_y, dist_x);
    linear_error = std::sqrt((dist_x * dist_x) + (dist_y * dist_y));
    angular_error = path_yaw - current_yaw;
    angular_error = std::atan2(std::sin(angular_error), std::cos(angular_error));
}

void PurePursuitController::update_control() {
    if (init_ang_adj == false) {
        // Fixing Initial Alignment
        init_ang_adj = pid_controller.update(angular_error, output_cmd_vel);
        vel_pub_->publish(output_cmd_vel);
    }
    else {
        pure_pursuit();
    }
}

void PurePursuitController::pure_pursuit() {
    output_cmd_vel.linear.x = max_speed;
    output_cmd_vel.angular.z = heading_const * angular_error;

    vel_pub_->publish(output_cmd_vel);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}