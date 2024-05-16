#ifndef PIDARCHITECTURES_H
#define PIDARCHITECTURES_H

#include "PIDlib.h"

class PIDController {
private:
    PID pidController;

public:
    PIDController() {}
    PIDController(PID::Builder builder) : pidController(builder.build()) {}

    void update(double kp, double ki, double kd)
    {
        pidController.setKp(kp);
        pidController.setKi(ki);
        pidController.setKd(kd);
    }

    void getParameters(double &kp, double &ki, double &kd)
    {
        kp = pidController.getKp();
        ki = pidController.getKi();
        kd = pidController.getKd();
    }

    void setDT(double dt)
    {
        pidController.set_dt(dt);
    }

    double control(double setpoint, double measurement) {
        pidController.compute(setpoint, measurement);
        return pidController.getOutput();
    }
};

class PDController {
private:
    PID pdController;

public:
    PDController() {}
    PDController(PID::Builder builder) : pdController(builder.build()) {}

    void update(double kp, double kd)
    {
        pdController.setKp(kp);
        pdController.setKd(kd);
    }

    void getParameters(double &kp, double &kd)
    {
        kp = pdController.getKp();
        kd = pdController.getKd();
    }

    void setDT(double dt)
    {
        pdController.set_dt(dt);
    }

    double control(double setpoint, double measurement) {
        pdController.compute(setpoint, measurement);
        return pdController.getOutput();
    }
};

class CascadePDPIController
{
private:
    PID pdController;
    PID piController;

public:
    CascadePDPIController() {}
    CascadePDPIController(PID::Builder builder_pd, PID::Builder builder_pi)
        : pdController(builder_pd.build()),
          piController(builder_pi.build()) {}

    double control(double setpoint, double pd_measurement, double pi_measurement)
    {
        pdController.compute(setpoint, pd_measurement);
        double pd_output = pdController.getOutput();
        piController.compute(pd_output, pi_measurement);
        return piController.getOutput();
    }

    void update(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        pdController.setKp(kp_pd);
        pdController.setKd(kd_pd);
        piController.setKp(kp_pi);
        piController.setKi(ki_pi);
    }

    void getParameters(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        kp_pd = pdController.getKp();
        kd_pd = pdController.getKd();
        kp_pi = piController.getKp();
        ki_pi = piController.getKi();
    }

    void setDT(double dt)
    {
        pdController.set_dt(dt);
        piController.set_dt(dt);
    }
};

class ParallelPDPIController
{
private:
    PID pdController;
    PID piController;

public:
    ParallelPDPIController() {}
    ParallelPDPIController(PID::Builder builder_pd, PID::Builder builder_pi)
        : pdController(builder_pd.build()),
          piController(builder_pi.build()) {}

    // double control(double setpoint, double measurement)
    double control(double setpoint, double measurement, double setpoint_speed, double measurement_speed)
    {
        pdController.compute(setpoint, measurement);
        double pd_output = pdController.getOutput();
        // piController.compute(setpoint, measurement);
        piController.compute(setpoint_speed, measurement_speed);
        double pi_output = piController.getOutput();
        return pd_output + pi_output;
    }

    void update(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        pdController.setKp(kp_pd);
        pdController.setKd(kd_pd);
        piController.setKp(kp_pi);
        piController.setKi(ki_pi);
    }

    void getParameters(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        kp_pd = pdController.getKp();
        kd_pd = pdController.getKd();
        kp_pi = piController.getKp();
        ki_pi = piController.getKi();
    }

    void setDT(double dt)
    {
        pdController.set_dt(dt);
        piController.set_dt(dt);
    }
};

#endif // PIDARCHITECTURES_H