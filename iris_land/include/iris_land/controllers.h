#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "pidArchitectures.h"

struct Pose
{
    double x, y, z, theta;
};

struct Speed
{
    double vx, vy, vz, vtheta;
};

class PID_velocity_ctl
{
private:
    PIDController controller_x;
    PIDController controller_y;
    PIDController controller_z;
    PIDController controller_theta;

public:
    PID_velocity_ctl() {}

    PID_velocity_ctl(PID::Builder builder_x, PID::Builder builder_y, PID::Builder builder_z, PID::Builder builder_theta)
        : controller_x(builder_x),
          controller_y(builder_y),
          controller_z(builder_z),
          controller_theta(builder_theta) {}

    void update_x(double kp, double ki, double kd) { controller_x.update(kp, ki, kd); }
    void get_x(double &kp, double &ki, double &kd) { controller_x.getParameters(kp, ki, kd); }

    void update_y(double kp, double ki, double kd) { controller_y.update(kp, ki, kd); }
    void get_y(double &kp, double &ki, double &kd) { controller_y.getParameters(kp, ki, kd); }

    void update_z(double kp, double ki, double kd) { controller_z.update(kp, ki, kd); }
    void get_z(double &kp, double &ki, double &kd) { controller_z.getParameters(kp, ki, kd); }

    void update_theta(double kp, double ki, double kd) { controller_theta.update(kp, ki, kd); }
    void get_theta(double &kp, double &ki, double &kd) { controller_theta.getParameters(kp, ki, kd); }

    Speed control(const Pose &setpoint, const Pose &measurement)
    {
        Speed speed;
        speed.vx = controller_x.control(setpoint.x, measurement.x);
        speed.vy = controller_y.control(setpoint.y, measurement.y);
        speed.vz = controller_z.control(setpoint.z, measurement.z);
        speed.vtheta = controller_theta.control(setpoint.theta, measurement.theta);
        return speed;
    }
};

#endif // CONTROLLERS_H
