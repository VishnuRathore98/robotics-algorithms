#include <iostream>
#include <chrono>
#include <thread>
#include "PIDController.h"

int main() {
    // Create and configure the PID controller
    PIDController pid(1.2, 0.5, 0.1);  // Tune these gains as needed
    pid.setSetpoint(100.0);           // Desired motor speed (RPM)

    double currentSpeed = 0.0;        // Initial speed
    double dt = 0.1;                  // Time step (seconds)

    // Simulate 30 control cycles
    for (int i = 0; i < 30; ++i) {
        // 1. Get control signal from PID controller
        double controlSignal = pid.update(currentSpeed, dt);

        // 2. Simulate motor response (very simple model)
        // Control signal affects speed proportionally
        currentSpeed += controlSignal * dt;

        // 3. Print output
        std::cout << "Time: " << i * dt << "s"
                  << " | Control: " << controlSignal
                  << " | Speed: " << currentSpeed << " RPM\n";

        // 4. Wait for next cycle (simulate real time)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
