"""
Objective:
Balance an inverted pendulum in the upright position.

Control input: Force (u) applied to a cart.
"""

import numpy as np
from scipy.signal import place_poles

# System Parameters
M = 1.0  # Cart mass
m = 0.1  # Pendulum mass
l = 0.5  # Pendulum length
g = 9.81

# State-Space Matrices
A = np.array(
    [
        [0, 1, 0, 0],
        [0, 0, (m * g) / M, 0],
        [0, 0, 0, 1],
        [0, 0, ((M + m) * g) / (l * M), 0],
    ]
)

B = np.array([[0], [1 / M], [0], [1 / (l * M)]])

# Desired Closed-loop Poles
desired_poles = [-2, -2.5, -3, -3.5]

# Pole Placement to compute Gain K
place_obj = place_poles(A, B, desired_poles)
K = place_obj.gain_matrix
print("State Feedback Gain K:", K)

# Simulation Parameters
dt = 0.01  # Time step
total_time = 5  # seconds
steps = int(total_time / dt)

# Initial State: [position, velocity, angle, angular velocity]
x = np.array([[0.0], [0.0], [0.1], [0.0]])  # Small initial angle offset

# Simulation Loop (Euler Integration)
for step in range(steps):
    # Control Input
    u = -K @ x

    # System Dynamics: x_dot = A x + B u
    x_dot = A @ x + B * u

    # Euler Integration to update state
    x += x_dot * dt

    # Print state every 100 steps
    if step % 100 == 0:
        print(
            f"Time: {step*dt:.2f}s | Position: {x[0,0]:.3f} m | Angle: {x[2,0]:.3f} rad"
        )
