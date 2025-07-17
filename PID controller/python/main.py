from pid_controller import PIDController
import time

# Create a PID controller with tuned gains
pid = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=100)

# Simulated system value (e.g., motor speed starts at 0)
current_value = 0

# Time step in seconds (e.g., 100ms update rate)
dt = 0.1

# Run a simulation loop
for i in range(30):
    # Get PID output based on current system state
    control = pid.update(current_value, dt)

    # Simulate a simple system where output affects next value
    # (This is a very basic model with inertia)
    current_value += control * dt  # Assume control influences speed

    print(f"Time: {i*dt:.1f}s | Control: {control:.2f} | Speed: {current_value:.2f}")

    # Wait for next cycle (optional, just for realism)
    time.sleep(dt)
