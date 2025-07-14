#include <pid_controller.hpp>

PIDController::PIDController(float kp, float ki, float kd){
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
}

bool PIDController::update(double error, geometry_msgs::msg::Twist& output_cmd_vel) {
    if (std::fabs(error) > 0.1) {
        double val = kp * error;
        output_cmd_vel.angular.z = kp * error;
        return false;
    }
    else {
        output_cmd_vel.angular.z = 0;
        return true;
    }
}