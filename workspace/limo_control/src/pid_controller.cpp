#include <pid_controller.hpp>

PIDController::PIDController(float kp, float ki, float kd){
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
}

double PIDController::update(double error) {
    if (error > 0.1) {
        double val = kp * error;
        output_cmd_vel.angular.z = kp * error;
        return false;
    }
    else {
        output_cmd_vel.angular.z = 0;
        return true;
    }
}