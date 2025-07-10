#include "geometry_msgs/msg/twist.hpp"
#include <math.h>

class PIDController {
public:
    PIDController() 
    : lin_kp(0), lin_ki(0), lin_kd(0), ang_kp(0), ang_ki(0), ang_kd(0), fixing_yaw(false) {}
    PIDController(float lin_kp, float lin_ki, float lin_kd, float ang_kp, float ang_ki, float ang_kd);
    void update(double linear_error, double angular_error);
    void reset();
    geometry_msgs::msg::Twist output_cmd_vel;

private:
    
    float lin_kp = 0.0;
    float lin_ki = 0.0;
    float lin_kd = 0.0;
    float ang_kp = 0.0;
    float ang_ki = 0.0;
    float ang_kd = 0.0;
    
    bool fixing_yaw = true;

    double der_ang_err = 0.0;
    double prev_ang_err = 0.0;
    double int_ang_err = 0.0;
    double linear_output = 0.0;
    double angular_output = 0.0;
    double dt = 0.1;
};