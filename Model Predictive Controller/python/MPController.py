# Simple Model Predictive Control (MPC) implementation
# Control a system (like a motor or temperature) to reach a desired target

# System model: x_next = x + u
# Goal: make x reach and stay at target (e.g., 10.0)
# We'll choose the best control input (u) over a horizon by minimizing a cost

target = 10.0  # Desired state (e.g., position, angle, temperature)
horizon = 5  # Prediction steps (how far we look into the future)
control_range = [-1, 0, 1]  # Possible control actions (like PWM or motor speed)
Q = 1.0  # Weight for error (state deviation from target)
R = 0.1  # Weight for control effort (penalize large u)
dt = 1.0  # Time step (could be milliseconds or seconds)

x = 0.0  # Initial state (e.g., temperature = 0°C)

# Simulate 20 control steps
for t in range(20):
    best_sequence = None
    best_cost = float("inf")
    best_u = 0

    # Brute-force all possible control sequences of length = horizon
    from itertools import product

    for u_seq in product(control_range, repeat=horizon):
        x_pred = x
        cost = 0

        # Simulate the future with this control sequence
        for k in range(horizon):
            u = u_seq[k]
            x_pred = x_pred + u * dt  # Simple model: x_next = x + u * dt
            error = x_pred - target

            # Cost = error squared + small penalty on control effort
            cost += Q * (error**2) + R * (u**2)

        # Keep the sequence with the lowest cost
        if cost < best_cost:
            best_cost = cost
            best_sequence = u_seq
            best_u = u_seq[0]  # Only apply first control input!

    # Apply best first control input to actual system
    x = x + best_u * dt

    # Print what's happening
    print(f"Time {t}, State: {x:.2f}, Control: {best_u}, Cost: {best_cost:.2f}")
