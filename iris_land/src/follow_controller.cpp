#include "follow_controller.h"

Follow_Controller::Follow_Controller()
{
    setpoint.x = 0;
    setpoint.y = 0;
    setpoint.z = 1;
    setpoint.theta = 0;

    PID::Builder builder;
    builder.setDt(0.05);
    builder.setOutMax(1);
    builder.setConditionalIntegration(true);

    PID_velocity_ctl pid_Controller(
        builder,
        builder,
        builder,
        builder);
    pidController = pid_Controller;
}

Follow_Controller::~Follow_Controller()
{
}

void Follow_Controller::print_parameters()
{
    cout << "Follow_Controller: " << endl;
    cout << "\tx: " << setpoint.x << "\ty: " << setpoint.y
         << "\tz: " << setpoint.z << "\ttheta: " << setpoint.theta << endl;

    double kp, ki, kd;
    kp = ki = kd = 0;
    pidController.get_x(kp, ki, kd);
    cout << "\tx_kp: " << kp << "\tx_ki: " << ki << "\tx_kd: " << kd << endl;

    pidController.get_y(kp, ki, kd);
    cout << "\ty_kp: " << kp << "\ty_ki: " << ki << "\ty_kd: " << kd << endl;

    pidController.get_z(kp, ki, kd);
    cout << "\tz_kp: " << kp << "\tz_ki: " << ki << "\tz_kd: " << kd << endl;

    pidController.get_theta(kp, ki, kd);
    cout << "\tt_kp: " << kp << "\tt_ki: " << ki << "\tt_kd: " << kd << endl;
}

void Follow_Controller::update_parameters(iris_land::controllers_gain newParameters)
{
    pidController.update_x(newParameters.pid_ctrl.x.p_gain,
                           newParameters.pid_ctrl.x.i_gain,
                           newParameters.pid_ctrl.x.d_gain);
    pidController.update_y(newParameters.pid_ctrl.y.p_gain,
                           newParameters.pid_ctrl.y.i_gain,
                           newParameters.pid_ctrl.y.d_gain);
    pidController.update_z(newParameters.pid_ctrl.z.p_gain,
                           newParameters.pid_ctrl.z.i_gain,
                           newParameters.pid_ctrl.z.d_gain);
    pidController.update_theta(newParameters.pid_ctrl.yaw.p_gain,
                               newParameters.pid_ctrl.yaw.i_gain,
                               newParameters.pid_ctrl.yaw.d_gain);

    setpoint.z = newParameters.altitude;
}

geometry_msgs::Twist Follow_Controller::get_velocity(geometry_msgs::PoseStamped poseStamped)
{
    geometry_msgs::Twist velocity;

    if (poseStamped.header.stamp.isZero())
    {
        return velocity;
    }

    Pose measurement;
    measurement.x = poseStamped.pose.position.x;
    measurement.y = poseStamped.pose.position.y;
    measurement.z = poseStamped.pose.position.z;
    measurement.theta = get_yaw(poseStamped.pose.orientation);

    Speed vel;
    vel = pidController.control(setpoint, measurement);

    velocity.linear.x = -vel.vy;
    velocity.linear.y = vel.vx;
    velocity.linear.z = vel.vz;
    velocity.angular.z = vel.vtheta;

    return velocity;
}

double Follow_Controller::calc_vel(double valor_in)
{
    const double MAX = 1.5;    // Valor máximo permitido
    const double MIN = 0.1;    // Valor mínimo permitido
    double valorRetorno = 0.6; // Valor a ser retornado
    double return_value;
    double valor = valor_in;
    if (valor_in < 0)
        valor = -valor;

    if (valor >= MIN && valor <= MAX)
    {
        return_value = valorRetorno;
    }
    else if (valor < MIN)
    {
        double slope = valorRetorno / (MIN - 0.0); // Inclinação da reta
        double intercept = -slope * 0.0;           // Intercepto da reta
        return_value = slope * valor + intercept;
    }
    else if (valor > MAX)
    {
        if (valor == 2 * MAX)
        {
            return_value = 0.0;
        }
        else
        {
            double slope = -valorRetorno / (2 * MAX - MAX); // Inclinação da reta
            double intercept = -slope * MAX + valorRetorno; // Intercepto da reta
            return_value = slope * valor + intercept;
        }
    }
    else
    {
        // Trate outros casos se necessário
        return_value = valor;
    }
    if (return_value < 0)
        return_value = 0;

    if (valor_in > 0)
        return_value = -return_value;

    return return_value;
}