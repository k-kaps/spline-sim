#include "geometry_msgs/msg/twist.hpp"
#include <math.h>

class PIDController {
public:
    PIDController() : fixing_yaw(true) {};
    PIDController(float kp, float ki, float kd);
    bool update(double error, geometry_msgs::msg::Twist& output_cmd_vel);
    void reset();

private:
    
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    
    bool fixing_yaw = true;

    double der_ang_err = 0.0;
    double prev_ang_err = 0.0;
    double int_ang_err = 0.0;
    double linear_output = 0.0;
    double angular_output = 0.0;
    double dt = 0.1;
};