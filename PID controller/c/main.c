#include <stdio.h>
#include <unistd.h>  // For sleep on Unix/Linux (optional for delay)
#include <PIDController.h>

int main() {
    PIDController pid;

    // Initialize PID with kp, ki, kd, and setpoint (target speed)
    pid_init(&pid, 1.2, 0.5, 0.1, 100.0);  // Target speed = 100 RPM

    double current_speed = 0.0;  // Initial motor speed
    double dt = 0.1;             // Time step (in seconds)

    // Simulate 30 update cycles
    for (int i = 0; i < 30; i++) {
        // Get the PID output (e.g., PWM control signal)
        double control_signal = pid_update(&pid, current_speed, dt);

        // Simulate system response: control affects motor speed
        // This is a very simplified model
        current_speed += control_signal * dt;

        // Print the output to observe how it changes over time
        printf("Time: %.1fs | Control: %.2f | Speed: %.2f\n", i * dt, control_signal, current_speed);

        // Sleep for 100ms to simulate real time (optional)
        usleep(100000);  // 100,000 microseconds = 100 ms
    }

    return 0;
}
