"""
# Let's say we have:

# Desired tank level = 100

# Valve disturbance = known outflow

# Control variable = pump speed (inflow)
"""

# Simple Feedforward + PID Example

# Constants
desired_level = 100
Kp = 2.0
Ki = 0.5
Kd = 0.1

# Initial Conditions
current_level = 100
integral = 0
previous_error = 0

# Known disturbance (liters/sec)
valve_outflow = 5  # Feedforward known disturbance


# Feedforward calculation (compensate disturbance)
def feedforward(valve_outflow):
    # Assume pump can fully compensate
    return valve_outflow


# PID calculation
def pid_control(error, integral, derivative):
    return Kp * error + Ki * integral + Kd * derivative


# Simulation loop
for t in range(1, 11):  # Simulate 10 time steps
    # Error calculation
    error = desired_level - current_level
    integral += error
    derivative = error - previous_error

    # Calculate control actions
    u_ff = feedforward(valve_outflow)
    u_pid = pid_control(error, integral, derivative)

    # Total control action (inflow)
    inflow = u_ff + u_pid

    # Update water level (simplified dynamics)
    current_level += inflow - valve_outflow

    # Save error for next derivative calculation
    previous_error = error

    print(f"Time {t}: Water Level = {current_level:.2f} liters")


"""
What’s Happening?

Feedforward part compensates known disturbance (valve outflow).

PID part cleans up small errors due to inaccuracies.

The combination gives faster and smoother control.
"""
