// PIDController.cpp
#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), setpoint(0), integral(0), previousError(0) {}

void PIDController::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

double PIDController::update(double currentValue, double dt) {
    // 1. Calculate the current error
    double error = setpoint - currentValue;

    // 2. Accumulate the integral (error over time)
    integral += error * dt;

    // 3. Calculate the derivative (rate of error change)
    double derivative = (error - previousError) / dt;

    // 4. Combine all terms to compute the control output
    double output = (kp * error) + (ki * integral) + (kd * derivative);

    // 5. Store current error for next time
    previousError = error;

    return output;
}
