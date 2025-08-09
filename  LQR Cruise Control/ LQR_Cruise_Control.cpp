#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

// Solve Continuous Algebraic Riccati Equation (simple scalar version for 1x1 case)
double solveCARE(double A, double B, double Q, double R) {
    // For 1x1 system, CARE reduces to: 2*A*P - (P^2 * B^2)/R + Q = 0
    // Rearrange into: -(B^2/R) * P^2 + 2*A*P + Q = 0  (quadratic in P)
    double a = -(B * B / R);
    double b = 2 * A;
    double c = Q;

    double disc = b * b - 4 * a * c;
    if (disc < 0) {
        std::cerr << "Negative discriminant in CARE" << std::endl;
        return 0;
    }

    // Positive root for stabilizing solution
    double P1 = (-b + std::sqrt(disc)) / (2 * a);
    double P2 = (-b - std::sqrt(disc)) / (2 * a);
    return (P1 > 0) ? P1 : P2;
}

int main() {
    // System parameters
    double m = 1000.0; // kg
    double b = 50.0;   // Ns/m

    // State-space matrices (scalar form)
    double A = -b / m;
    double B = 1.0 / m;

    // LQR weights
    double Q = 1.0;    // Penalize speed error
    double R = 0.01;   // Penalize control effort

    // Solve CARE to get P
    double P = solveCARE(A, B, Q, R);

    // Compute LQR gain
    double K = (1.0 / R) * (B * P);

    std::cout << "LQR Gain K: " << K << std::endl;

    // Simulation parameters
    double dt = 0.1;       // time step (s)
    double v = 0.0;        // initial speed
    double v_desired = 20; // m/s

    for (double t = 0; t <= 20; t += dt) {
        double error = v - v_desired;
        double u = -K * error; // LQR control

        // System dynamics: dv/dt = A*v + B*u
        double dv = A * v + B * u;
        v += dv * dt;

        std::cout << "t=" << t << "s, v=" << v << " m/s, u=" << u << " N" << std::endl;
    }

    return 0;
}
