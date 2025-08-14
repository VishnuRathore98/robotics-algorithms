"""
Sliding Mode Control

Problem Setup

We want the motor speed to be 50 rad/s

There’s some unknown push/pull (disturbance) on the motor

The SMC should keep the speed close to 50 despite this

"""

import numpy as np
import matplotlib.pyplot as plt

# Time setup
dt = 0.001
time = np.arange(0, 3, dt)

# Motor parameters
J = 0.01  # inertia
b = 0.1  # damping

# Desired speed
target_speed = 50.0  # rad/s

# SMC parameters
lambda_s = 20.0
K = 0.5

# Initial values
speed = 0.0
speed_change = 0.0

# Data logs
speeds = []
disturbances = []

for t in time:
    # Disturbance (changes halfway)
    disturbance = 0.2 if t < 1.5 else -0.4

    # Error and sliding surface
    error = target_speed - speed
    error_dot = -speed_change
    s = lambda_s * error + error_dot

    # Simple SMC control
    control = J * (-lambda_s * error_dot) + b * target_speed + K * np.sign(s)

    # Motor physics update
    speed_change = (control - b * speed + disturbance) / J
    speed += speed_change * dt

    # Save data
    speeds.append(speed)
    disturbances.append(disturbance)

# Plot speed result
plt.plot(time, speeds, label="Motor Speed")
plt.axhline(target_speed, color="k", linestyle="--", label="Target Speed")
plt.xlabel("Time (s)")
plt.ylabel("Speed (rad/s)")
plt.legend()
plt.grid()
plt.show()
