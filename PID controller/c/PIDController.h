#include <stdio.h>

// Define a structure to hold all PID-related variables
typedef struct {
    double kp;               // Proportional gain
    double ki;               // Integral gain
    double kd;               // Derivative gain

    double setpoint;         // Desired target value

    double integral;         // Integral accumulator
    double previous_error;   // Stores last error to compute derivative
} PIDController;

// Initialize the PID controller
void pid_init(PIDController *pid, double kp, double ki, double kd, double setpoint) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0;
    pid->previous_error = 0.0;
}

// Update function that computes the new control output
double pid_update(PIDController *pid, double current_value, double dt) {
    // Calculate the error between desired and current value
    double error = pid->setpoint - current_value;

    // Accumulate the error for the integral term
    pid->integral += error * dt;

    // Calculate rate of error change for the derivative term
    double derivative = (error - pid->previous_error) / dt;

    // Calculate total PID output by combining all three terms
    double output = pid->kp * error +
                    pid->ki * pid->integral +
                    pid->kd * derivative;

    // Store the current error for next derivative calculation
    pid->previous_error = error;

    // Return the control output (e.g., to apply to a motor)
    return output;
}
