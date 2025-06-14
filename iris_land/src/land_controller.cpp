#include "land_controller.h"

Land_Controller::Land_Controller()
{
    setpoint.x = 0;
    setpoint.y = 0;
    setpoint.z = 2;
    setpoint.theta = 0;
    controller_mode = 0;
    distance_threshold = 0.2;
    angular_threshold = 0.2;

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

Land_Controller::~Land_Controller()
{
}

void Land_Controller::print_parameters()
{
    cout << "Land_Controller: " << endl;
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

void Land_Controller::update_parameters(iris_land::controllers_gain newParameters)
{
    pidController.update_x( newParameters.pid_ctrl.x.p_gain,
                            newParameters.pid_ctrl.x.i_gain,
                            newParameters.pid_ctrl.x.d_gain);
    pidController.update_y( newParameters.pid_ctrl.y.p_gain,
                            newParameters.pid_ctrl.y.i_gain,
                            newParameters.pid_ctrl.y.d_gain);
    pidController.update_z( newParameters.pid_ctrl.z.p_gain,
                            newParameters.pid_ctrl.z.i_gain,
                            newParameters.pid_ctrl.z.d_gain);
    pidController.update_theta( newParameters.pid_ctrl.yaw.p_gain,
                                newParameters.pid_ctrl.yaw.i_gain,
                                newParameters.pid_ctrl.yaw.d_gain);
}

void Land_Controller::reset_altitude(double altitude)
{
    setpoint.z = altitude;
}

bool Land_Controller::completed_approach()
{
    if(setpoint.z < 0.7)
        return true;
    return false;
}

geometry_msgs::Twist Land_Controller::get_velocity(geometry_msgs::PoseStamped poseStamped)
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

    double distance = calculate_distance(measurement, setpoint);
    double angle_distance = measurement.theta - setpoint.theta;
    if (angle_distance < 0)
        angle_distance *= -1;

    cout << "distance:\t" << distance << "\t" << distance_threshold << endl;
    cout << "angle_distance:\t" << angle_distance << "\t" << angular_threshold << endl;

    if ((distance <= distance_threshold) && (angle_distance <= angular_threshold))
    {
        cout << "******* chegou *******" << endl;
        setpoint.z = update_altitude(setpoint.z);
        cout << setpoint.z << ": " << update_altitude(setpoint.z) << endl;
    }

    Speed vel = get_align_velocity(measurement);

    velocity.linear.x = -vel.vy;
    velocity.linear.y = vel.vx;
    velocity.linear.z = vel.vz;
    velocity.angular.z = vel.vtheta;
    return velocity;
}

Speed Land_Controller::get_align_velocity(Pose poseMeasurement)
{
    Speed vel;
    vel = pidController.control(setpoint, poseMeasurement);
    return vel;
}

double Land_Controller::calc_vel(double valor_in)
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

double Land_Controller::calculate_distance(const Pose &point1, const Pose &point2)
{
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2) + pow(point2.z - point1.z, 2));
}

double Land_Controller::update_altitude(double altitude)
{
    double return_value;
    return_value = altitude - 0.01;
    return return_value;
}