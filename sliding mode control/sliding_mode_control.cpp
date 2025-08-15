#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

int main() {
    // Time setup
    double dt = 0.001;
    double T = 3.0;
    int steps = static_cast<int>(T / dt);

    // Motor parameters
    double J = 0.01;   // inertia
    double b = 0.1;    // damping

    // Desired speed
    double target_speed = 50.0; // rad/s

    // SMC parameters
    double lambda_s = 20.0;
    double K = 0.5;

    // Initial state
    double speed = 0.0;
    double speed_change = 0.0;

    // Data storage for plotting
    std::vector<double> time_log;
    std::vector<double> speed_log;
    std::vector<double> disturbance_log;

    for (int i = 0; i < steps; i++) {
        double t = i * dt;

        // Disturbance (changes halfway)
        double disturbance = (t < 1.5) ? 0.2 : -0.4;

        // Error and sliding surface
        double error = target_speed - speed;
        double error_dot = -speed_change;
        double s = lambda_s * error + error_dot;

        // SMC control law
        double control = J * (-lambda_s * error_dot) + b * target_speed + K * ((s > 0) - (s < 0));

        // Motor dynamics update
        speed_change = (control - b * speed + disturbance) / J;
        speed += speed_change * dt;

        // Save data
        time_log.push_back(t);
        speed_log.push_back(speed);
        disturbance_log.push_back(disturbance);
    }

    // Save to CSV for plotting in Python/Excel
    std::ofstream file("smc_motor_data.csv");
    file << "time,speed,disturbance\n";
    for (size_t i = 0; i < time_log.size(); i++) {
        file << time_log[i] << "," << speed_log[i] << "," << disturbance_log[i] << "\n";
    }
    file.close();

    std::cout << "Simulation complete. Data saved to smc_motor_data.csv\n";
    std::cout << "You can plot this in Python or Excel.\n";

    return 0;
}
