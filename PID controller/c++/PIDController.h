// PIDController.h
#pragma once

class PIDController {
public:
    PIDController(double kp, double ki, double kd);

    void setSetpoint(double setpoint);
    double update(double currentValue, double dt);

private:
    double kp;               // Proportional gain
    double ki;               // Integral gain
    double kd;               // Derivative gain

    double setpoint;         // Desired value
    double integral;         // Integral accumulator
    double previousError;    // Last error value for derivative
};
