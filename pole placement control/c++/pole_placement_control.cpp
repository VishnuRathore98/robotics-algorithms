/*
*   Implement pole placement in C++ for a linear system
*/

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

int main() {
    // Define system matrices
    Matrix2d A;
    A << 0, 1,
        -2, -3;

    Vector2d B;
    B << 0,
         1;

    // Desired poles: s = -2 and -3 => Desired char poly: s^2 + 5s + 6
    // So: A_desired = A^2 + 5*A + 6*I
    Matrix2d I = Matrix2d::Identity();
    Matrix2d A2 = A * A;
    Matrix2d phiA = A2 + 5*A + 6*I;

    // Controllability matrix: [B, A*B]
    Matrix2d ctrb;
    ctrb << B, A*B;

    // Inverse of controllability matrix
    Matrix2d ctrb_inv = ctrb.inverse();

    // Row vector: [0 1]
    RowVector2d row = RowVector2d::Zero();
    row(1) = 1;

    // Compute gain K using Ackermann’s formula
    RowVector2d K = row * ctrb_inv * phiA;

    std::cout << "Pole Placement Gain K: " << K << std::endl;

    return 0;
}
