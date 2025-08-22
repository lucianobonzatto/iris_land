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

#endif // PIDARCHITECTURES_H