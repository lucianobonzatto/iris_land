#ifndef PIDCLASS_PIDLIB_H
#define PIDCLASS_PIDLIB_H

/*
 * Library Name: PID Library
 * Version: 1.0.0
 * Date: 07/05/2023
 * Author: João Afonso Braun Neto, Paulo Costa, José Lima
 * Contact: jabraunneto1@gmail.com
 *
 * Description:
 * The PID Library is a C++ implementation of a PID (Proportional-Integral-Derivative) controller.
 * The library includes features like conditional integration, feedforward control, and the ability
 * to handle angular inputs.
 *
 * Copyright (c) 2023 Your Name. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <tuple>

const double PI = M_PI;
const double NEG_PI = -M_PI;

enum class DebugLevel {
    BasicInfo,
    ParameterInfo,
    DetailedInfo
};

// PID is a class that represents a Proportional Integral Derivative (PID) controller.

class PID {


private:

    // The following private members are the different parameters and intermediate
    // variables used in the PID control algorithm.

    double Kp, Ki, Kd, Kf;  // The proportional, integral, derivative and feedforward gains
    double dt;  // The time difference between updates
    // More private variables here
    double estimated_process_variable, reference_process_variable;
    double error, last_error, integral_error, differential_last_error, last_process_variable;
    double output, output_max, output_min;
    bool is_derivative_on_measurement;
    bool is_conditional_integration;
    bool is_feedforward_enabled;
    bool is_angular_input;

    // This method ensures compatibility with Python's modulo operation.
    double pythonModuloCompability(const double a, const double n);
    // This method normalizes an angle within the range [lower_bound, upper_bound).
    double normalize_angle(double angle, const double lower_bound = NEG_PI, const double upper_bound = PI,
                           const bool symmetric = false);

    // Private constructor can be accessed by Builder
    PID(double Kp, double Ki, double Kd, double Kf, double dt, double out_max, double out_min,
        bool derivative_on_measurement, bool conditional_integration, bool feedforward_enabled, bool angular_input);

public:
    PID(){}
// Builder is a nested public class used to construct PID object using Builder Pattern.
    class Builder {
    private:
        // Builder's private members are the parameters to be used in the PID object.
        double Kp = 0.0, Ki = 0.0, Kd = 0.0, Kf = 0.0, dt = 0.05, out_max = 255.0, out_min = -255.0;
        bool derivative_on_measurement = false, conditional_integration = false, feedforward_enabled = false, angular_input = false;

    public:
        // The Builder's public methods are setters for each of its private members.
        // Each method returns a reference to the Builder object to allow method chaining.
        Builder& setKp(double Kp) { this->Kp = Kp; return *this; }
        Builder& setKi(double Ki) { this->Ki = Ki; return *this; }
        Builder& setKd(double Kd) { this->Kd = Kd; return *this; }
        Builder& setKf(double Kf) { this->Kf = Kf; return *this; }
        Builder& setDt(double dt) { this->dt = dt; return *this; }
        Builder& setOutMax(double out_max) { this->out_max = out_max; return *this; }
        Builder& setOutMin(double out_min) { this->out_min = out_min; return *this; }
        Builder& setDerivativeOnMeasurement(bool derivative_on_measurement) { this->derivative_on_measurement = derivative_on_measurement; return *this; }
        Builder& setConditionalIntegration(bool conditional_integration) { this->conditional_integration = conditional_integration; return *this; }
        Builder& setFeedforwardEnabled(bool feedforward_enabled) { this->feedforward_enabled = feedforward_enabled; return *this; }
        Builder& setAngularInput(bool angular_input) { this->angular_input = angular_input; return *this; }

        // This method builds and returns a PID object.
        PID build() {
            return PID(Kp, Ki, Kd, Kf, dt, out_max, out_min, derivative_on_measurement, conditional_integration, feedforward_enabled, angular_input);
        }
    };

    // Make Builder a friend of PID so it can access the private constructor.
    friend class Builder;

    // Methods

    // compute() updates the PID controller with the latest inputs and calculates the new output.
    void compute(double new_reference_process_variable, double new_estimated_process_variable);

    // reset() clears the errors and the output of the PID controller.
    void reset();

    // Enable or disable certain features
    void enableFeedforward(bool use_feedforward);
    void enableAngularInput(bool angular_input);

    // Getters
    double getError() const;
    double getIntegralError() const;
    double getDerivativeError() const;
    double getOutput() const;
    double getInterval() const;
    std::tuple<double, double, double, double> getParameters() const;
    double getKp() const;
    double getKi() const;
    double getKd() const;
    double getKf() const;

    // Setters
    void setParameters(double Kp, double Ki, double Kd, double Kf);
    void setKp(double Kp);
    void setKi(double Ki);
    void setKd(double Kd);
    void setKf(double Kf);
    void set_dt(double dt);
    void setOutputLimits(double out_max, double out_min);

    // Debugging
    void debug(DebugLevel level) const;
};

#endif //PIDCLASS_PIDLIB_H