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

    float angular_kp = 0.08;
    float angular_ki = 0.01;
    float angular_kd = 0.01;

    pid_controller = PIDController(angular_kp, angular_ki, angular_kd);
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
        geometry_msgs::msg::Twist cmd_vel;
        vel_publisher->publish(cmd_vel);
    }
}

bool PurePursuitController::store_target_pose() {
    for (int i = start_pt; i < robot_path.poses.size(); i++) {
        double dist_x = robot_path.poses[i].pose.position.x - this->current_pose.position.x;
        double dist_y = robot_path.poses[i].pose.position.y - this->current_pose.position.y;
        double dist = std::sqrt((dist_x * dist_x) + (dist_y * dist_y));
        if (dist > this->ld) {
            target_pose = robot_path.poses[i];
            prev_start_pt = start_pt;
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

void PurePursuitController::compute_cmd_vel_carrot() {
    double v = 0.1;  // fixed speed
    double k = 3;  // heading gain

    double dx = target_pose.pose.position.x - current_pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.position.y;

    double path_yaw = std::atan2(dy, dx);
    double robot_yaw = tf2::getYaw(current_pose.orientation);
    double heading_error = path_yaw - robot_yaw;

    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));  // normalize

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = k * heading_error;

    vel_publisher->publish(cmd);
}


void PurePursuitController::compute_cmd_vel() {
    // if this is the first point we are heading for and we haven't met the desired heading
    if (pid_adjustment == false){
        // only do heading correction first
        pid_adjustment = pid_controller.update(angular_error);
        RCLCPP_INFO(this->get_logger(), "the output val: %f", pid_controller.output_cmd_vel.angular.z);
        vel_publisher->publish(pid_controller.output_cmd_vel);
    }
    else {
        RCLCPP_INFO(this->get_logger(), "We made it out");
        compute_cmd_vel_carrot();
    }
}

void PurePursuitController::compute_errors() {
    double dist_x = this->target_pose.pose.position.x - this->current_pose.position.x;
    double dist_y = this->target_pose.pose.position.y - this->current_pose.position.y;
    double yaw = tf2::getYaw(this->current_pose.orientation);
    double desired_heading = std::atan2(dist_y, dist_x);

    this->linear_error = std::sqrt((dist_x * dist_x) + (dist_y * dist_y));
    this->angular_error = desired_heading - yaw;
    RCLCPP_INFO(this->get_logger(), "lin: %f, ang: %f", this->linear_error, this->angular_error);
    normalize_angular_error();
    RCLCPP_INFO(this->get_logger(), "lin: %f, ang: %f", this->linear_error, this->angular_error);
}

void PurePursuitController::normalize_angular_error() {
    this->angular_error = std::atan2(std::sin(this->angular_error), std::cos(this->angular_error));
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