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

class PD_velocity_ctl
{
private:
    PDController controller_x;
    PDController controller_y;
    PDController controller_z;
    PDController controller_theta;

public:
    PD_velocity_ctl() {}

    PD_velocity_ctl(PID::Builder builder_x, PID::Builder builder_y, PID::Builder builder_z, PID::Builder builder_theta)
        : controller_x(builder_x),
          controller_y(builder_y),
          controller_z(builder_z),
          controller_theta(builder_theta) {}

    void update_x(double kp, double kd) { controller_x.update(kp, kd); }
    void get_x(double &kp, double &kd) { controller_x.getParameters(kp, kd); }

    void update_y(double kp, double kd) { controller_y.update(kp, kd); }
    void get_y(double &kp, double &kd) { controller_y.getParameters(kp, kd); }

    void update_z(double kp, double kd) { controller_z.update(kp, kd); }
    void get_z(double &kp, double &kd) { controller_z.getParameters(kp, kd); }

    void update_theta(double kp, double kd) { controller_theta.update(kp, kd); }
    void get_theta(double &kp, double &kd) { controller_theta.getParameters(kp, kd); }

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

class cascade_velocity_ctl
{
private:
    CascadePDPIController controller_x;
    CascadePDPIController controller_y;
    CascadePDPIController controller_z;
    CascadePDPIController controller_theta;

public:
    cascade_velocity_ctl() {}
    cascade_velocity_ctl(PID::Builder builder_x_pd, PID::Builder builder_x_pi, PID::Builder builder_y_pd, PID::Builder builder_y_pi,
                               PID::Builder builder_z_pd, PID::Builder builder_z_pi, PID::Builder builder_theta_pd, PID::Builder builder_theta_pi)
        : controller_x(builder_x_pd, builder_x_pi),
          controller_y(builder_y_pd, builder_y_pi),
          controller_z(builder_z_pd, builder_z_pi),
          controller_theta(builder_theta_pd, builder_theta_pi) {}

    void update_x(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        controller_x.update(kp_pd, kd_pd, kp_pi, ki_pi);
    }
    void get_x(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        controller_x.getParameters(kp_pd, kd_pd, kp_pi, ki_pi);
    }

    void update_y(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        controller_y.update(kp_pd, kd_pd, kp_pi, ki_pi);
    }
    void get_y(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        controller_y.getParameters(kp_pd, kd_pd, kp_pi, ki_pi);
    }

    void update_z(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        controller_z.update(kp_pd, kd_pd, kp_pi, ki_pi);
    }
    void get_z(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        controller_z.getParameters(kp_pd, kd_pd, kp_pi, ki_pi);
    }

    void update_theta(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        controller_theta.update(kp_pd, kd_pd, kp_pi, ki_pi);
    }
    void get_theta(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        controller_theta.getParameters(kp_pd, kd_pd, kp_pi, ki_pi);
    }

    Speed control(const Pose &setpoint, const Pose &measurement, const Speed &est_speed)
    {
        Speed speed;
        speed.vx = controller_x.control(setpoint.x, measurement.x, est_speed.vx);
        speed.vy = controller_y.control(setpoint.y, measurement.y, est_speed.vy);
        speed.vz = controller_z.control(setpoint.z, measurement.z, est_speed.vz);
        speed.vtheta = controller_theta.control(setpoint.theta, measurement.theta, est_speed.vtheta);
        return speed;
    }
};

class parallel_velocity_ctl
{
private:
    ParallelPDPIController controller_x;
    ParallelPDPIController controller_y;
    ParallelPDPIController controller_z;
    ParallelPDPIController controller_theta;

public:
    parallel_velocity_ctl() {}
    parallel_velocity_ctl(PID::Builder builder_x_pd, PID::Builder builder_x_pi,
                                PID::Builder builder_y_pd, PID::Builder builder_y_pi,
                                PID::Builder builder_z_pd, PID::Builder builder_z_pi,
                                PID::Builder builder_theta_pd, PID::Builder builder_theta_pi)
        : controller_x(builder_x_pd, builder_x_pi),
          controller_y(builder_y_pd, builder_y_pi),
          controller_z(builder_z_pd, builder_z_pi),
          controller_theta(builder_theta_pd, builder_theta_pi) {}

    void update_x(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        controller_x.update(kp_pd, kd_pd, kp_pi, ki_pi);
    }
    void get_x(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        controller_x.getParameters(kp_pd, kd_pd, kp_pi, ki_pi);
    }

    void update_y(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        controller_y.update(kp_pd, kd_pd, kp_pi, ki_pi);
    }
    void get_y(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        controller_y.getParameters(kp_pd, kd_pd, kp_pi, ki_pi);
    }

    void update_z(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        controller_z.update(kp_pd, kd_pd, kp_pi, ki_pi);
    }
    void get_z(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        controller_z.getParameters(kp_pd, kd_pd, kp_pi, ki_pi);
    }

    void update_theta(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        controller_theta.update(kp_pd, kd_pd, kp_pi, ki_pi);
    }
    void get_theta(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        controller_theta.getParameters(kp_pd, kd_pd, kp_pi, ki_pi);
    }

    // Speed control(const Pose& setpoint, const Pose& measurement) {
    //     Speed speed;
    //     speed.vx = controller_x.control(setpoint.x, measurement.x);
    //     speed.vy = controller_y.control(setpoint.y, measurement.y);
    //     speed.vz = controller_z.control(setpoint.z, measurement.z);
    //     speed.vtheta = controller_theta.control(setpoint.theta, measurement.theta);
    //     return speed;
    // }

    Speed control(const Pose &setpoint, const Pose &measurement, const Speed &setpoint_speed, const Speed &measurement_speed)
    {
        Speed speed;
        speed.vx = controller_x.control(setpoint.x, measurement.x, setpoint_speed.vx, measurement_speed.vx);
        speed.vy = controller_y.control(setpoint.y, measurement.y, setpoint_speed.vy, measurement_speed.vy);
        speed.vz = controller_z.control(setpoint.z, measurement.z, setpoint_speed.vz, measurement_speed.vz);
        speed.vtheta = controller_theta.control(setpoint.theta, measurement.theta, setpoint_speed.vtheta, measurement_speed.vtheta);
        return speed;
    }
};

#endif // CONTROLLERS_H
