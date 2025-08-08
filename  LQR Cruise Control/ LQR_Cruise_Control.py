import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are

# System parameters
m = 1000.0  # mass of vehicle (kg)
b = 50.0  # drag coefficient

# State-space model
A = np.array([[-b / m]])
B = np.array([[1 / m]])

# LQR weighting matrices
Q = np.array([[1]])  # Penalize speed error
R = np.array([[0.01]])  # Penalize control effort

# Solve Continuous Algebraic Riccati Equation (CARE)
P = solve_continuous_are(A, B, Q, R)

# Compute LQR gain
K = np.linalg.inv(R) @ B.T @ P
print("LQR Gain K:", K)

# Simulation parameters
dt = 0.1  # time step
time = np.arange(0, 50, dt)
v = 0.0  # initial speed
v_desired = 20  # desired speed (m/s)
x = np.array([[v]])
vel_history = []
u_history = []

for t in time:
    # Compute control input
    error = x[0, 0] - v_desired
    u = -K @ np.array([[error]])

    # System dynamics: dx/dt = Ax + Bu
    x_dot = A @ x + B * u
    x += x_dot * dt

    vel_history.append(x[0, 0])
    u_history.append(u[0, 0])

# Plot results
plt.figure()
plt.plot(time, vel_history, label="Speed (m/s)")
plt.axhline(v_desired, color="r", linestyle="--", label="Desired Speed")
plt.xlabel("Time (s)")
plt.ylabel("Speed")
plt.title("Cruise Control with LQR")
plt.legend()
plt.grid(True)
plt.show()

plt.figure()
plt.plot(time, u_history)
plt.xlabel("Time (s)")
plt.ylabel("Control Input (N)")
plt.title("Throttle Force over Time")
plt.grid(True)
plt.show()
