#include <pid_controller.hpp>

PIDController::PIDController(float lin_kp, float lin_ki, float lin_kd, float ang_kp, float ang_ki, float ang_kd){
    this->lin_kp = lin_kp;
    this->lin_kd = lin_kd;
    this->lin_ki = lin_ki;
    
    this->ang_kp = ang_kp;
    this->ang_kd = ang_kd;
    this->ang_ki = ang_ki;
}

// void PIDController::update(double lin_err, double ang_err) {
//     // --- Derivative term ---
//     der_ang_err = (ang_err - prev_ang_err) / dt;
//     prev_ang_err = ang_err;

//     // --- Integral term with anti-windup ---
//     int_ang_err += ang_err * dt;
//     int_ang_err = std::clamp(int_ang_err, -1.0, 1.0);

//     // --- PID outputs ---
//     double angular_pid = ang_kp * ang_err + ang_ki * int_ang_err + ang_kd * der_ang_err;
//     double linear_pid = lin_kp * lin_err;

//     // --- Hysteresis band thresholds ---
//     const double yaw_start_threshold = 0.2; // Enter fixing_yaw
//     const double yaw_stop_threshold = 0.2;  // Exit fixing_yaw

//     // --- Hysteresis state logic ---
//     if (std::fabs(ang_err) > yaw_start_threshold) {
//         fixing_yaw = true;
//     } else if (std::fabs(ang_err) < yaw_stop_threshold) {
//         fixing_yaw = false;
//     }

//     // --- Final output decision ---
//     if (fixing_yaw) {
//         linear_output = 0.0;
//         angular_output = angular_pid;
//     } else if (lin_err > 0.05) {
//         linear_output = linear_pid;
//         angular_output = angular_pid;
//     } else {
//         linear_output = 0.0;
//         angular_output = 0.0;
//     }

//     // --- Optional: Clamp velocities ---
//     double max_lin = 0.5;  // m/s
//     double max_ang = 1.0;  // rad/s
//     linear_output = std::clamp(linear_output, -max_lin, max_lin);
//     angular_output = std::clamp(angular_output, -max_ang, max_ang);

//     // --- Publish command ---
//     output_cmd_vel.linear.x = linear_output;
//     output_cmd_vel.angular.z = angular_output;
//}

void PIDController::update(double lin_err, double ang_err) {
    der_ang_err = (prev_ang_err - ang_err)/dt;
    int_ang_err += (int_ang_err * dt);
    int_ang_err = std::clamp(int_ang_err, -1.0, 1.0);
    prev_ang_err = ang_err;
    
    if (std::fabs(ang_err) > 0.05 && fixing_yaw == false) {
        output_cmd_vel.linear.x = 0.0;
        output_cmd_vel.angular.z = (ang_kp*ang_err) + (ang_kd*der_ang_err) + (ang_ki*int_ang_err);
    } 
    else if (lin_err > 0.05) {
        output_cmd_vel.linear.x = lin_kp*(lin_err);
        output_cmd_vel.angular.z = (ang_kp*ang_err) + (ang_kd*der_ang_err) + (ang_ki*int_ang_err);
    } 
    else if (std::fabs(ang_err) > 0.05) {
        fixing_yaw = true;
        output_cmd_vel.linear.x = 0.0;
        output_cmd_vel.angular.z = (ang_kp*ang_err) + (ang_kd*der_ang_err) + (ang_ki*int_ang_err);
    }
}