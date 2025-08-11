"""
We’ll use the mass-spring-damper system (a classic in robotics and control),
where the robot arm’s payload mass is unknown and changes mid-operation.

The adaptive controller will keep the system at a target position despite
the unknown and changing mass.
"""

"""
- Position stays near target before and after the mass change at 2.5s

- Estimated mass 𝑚 converges toward the new true mass

- Controller adapts without manual gain retuning
"""

import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.001
T = 5
time = np.arange(0, T, dt)

# True system parameters (unknown to controller)
m_true = 2.0  # kg
b = 1.0  # damping
k = 5.0  # spring constant

# Initial mass estimate
m_hat = 1.0  # kg (wrong at start)

# Controller gains
Kp, Kd = 50.0, 10.0
gamma_m = 5.0  # adaptation gain

# Target position
x_target = 1.0  # meters

# State variables
x, v = 0.0, 0.0  # position, velocity

# Data logs
x_log, m_hat_log, m_true_log = [], [], []

for t in time:
    # Error
    e = x_target - x
    edot = -v

    # Control law: PD + adaptive feedforward
    u = m_hat * (Kp * e + Kd * edot) + b * v + k * x

    # System dynamics (true mass)
    a = (u - b * v - k * x) / m_true
    v += a * dt
    x += v * dt

    # Adaptation law (gradient update)
    m_hat += gamma_m * e * (Kp * e + Kd * edot) * dt

    # Change mass halfway
    if abs(t - 2.5) < 1e-9:
        m_true = 3.0  # payload added

    # Log data
    x_log.append(x)
    m_hat_log.append(m_hat)
    m_true_log.append(m_true)

# Plot results
plt.figure(figsize=(8, 4))
plt.plot(time, x_log, label="Position")
plt.axhline(x_target, color="k", linestyle="--", label="Target")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.title("Adaptive Position Control")
plt.legend()
plt.grid()

plt.figure(figsize=(8, 4))
plt.plot(time, m_hat_log, label="Estimated Mass")
plt.plot(time, m_true_log, label="True Mass", linestyle="--")
plt.xlabel("Time [s]")
plt.ylabel("Mass [kg]")
plt.title("Adaptive Mass Estimation")
plt.legend()
plt.grid()

plt.show()
