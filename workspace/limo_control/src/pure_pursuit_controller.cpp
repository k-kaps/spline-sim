#include <pure_pursuit_controller.hpp>

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller") {
    // Initializing the Publisher
    vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    ld_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ld_pt", 10);

    // Initializaing the Subscribers
    path_subscriber = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&PurePursuitController::process_path, this, std::placeholders::_1));
    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PurePursuitController::process_odom, this, std::placeholders::_1));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    double linear_kp = 0.2;
    double linear_ki = 0.0;
    double linear_kd = 0.0;

    double angular_kp = 0.2;
    double angular_ki = 0.2;
    double angular_kd = 0.3;

    pid_controller = PIDController(linear_kp, linear_ki, linear_kd, angular_kp, angular_ki, angular_kd);
}

void PurePursuitController::process_path(const nav_msgs::msg::Path::SharedPtr path_msg) {
    robot_path = *path_msg;
}

void PurePursuitController::process_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    PurePursuitController::transform_robot_odom(odom_msg);
    RCLCPP_INFO(this->get_logger(), "The current pose is (x = %f, y = %f, z = %f, w = %f)", this->current_pose.position.x, this->current_pose.position.y, this->current_pose.orientation.z, this->current_pose.orientation.w);
    
    if (store_target_pose()){
        ld_publisher->publish(target_pose);
        compute_errors();
        compute_cmd_vel();
    }
    else {
        // we have reached the final target point
    }
}

bool PurePursuitController::store_target_pose() {
    for (int i = start_pt; i < robot_path.poses.size(); i++) {
        double dist_x = robot_path.poses[i].pose.position.x - this->current_pose.position.x;
        double dist_y = robot_path.poses[i].pose.position.y - this->current_pose.position.y;
        double dist = std::sqrt((dist_x * dist_x) + (dist_y * dist_y));
        if (dist > this->ld) {
            target_pose = robot_path.poses[i];
            start_pt  = i;
            return true;
        }
    }
    if (!this->robot_path.poses.empty()) {
        this->target_pose = this->robot_path.poses.back();
        return true;
    }
    return false;
}

void PurePursuitController::compute_cmd_vel() {
    pid_controller.update(linear_error, angular_error);
    vel_publisher->publish(pid_controller.output_cmd_vel);
}

void PurePursuitController::compute_errors() {
    double dist_x = this->target_pose.pose.position.x - this->current_pose.position.x;
    double dist_y = this->target_pose.pose.position.y - this->current_pose.position.y;
    double yaw = tf2::getYaw(this->target_pose.pose.orientation);
    double desired_heading = std::atan2(dist_y, dist_x);

    this->linear_error = std::sqrt((dist_x * dist_x) + (dist_y * dist_y));
    this->angular_error = desired_heading - yaw;
    RCLCPP_INFO(this->get_logger(), "lin: %f, ang: %f", this->linear_error, this->angular_error);
    normalize_angular_error();
    RCLCPP_INFO(this->get_logger(), "lin: %f, ang: %f", this->linear_error, this->angular_error);
}

void PurePursuitController::normalize_angular_error() {
    while (this->angular_error > M_PI) this->angular_error -= M_PI;
    while (this->angular_error < -M_PI) this->angular_error += M_PI;
}

void PurePursuitController::transform_robot_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    try {
        transform = tf_buffer->lookupTransform("world", "odom", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", "world", "odom", ex.what());
        return;
    }
    tf2::doTransform(odom_msg->pose.pose, this->current_pose, transform);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}