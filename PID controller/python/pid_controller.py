class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        """
        Initialize the PID controller with given gains:
        - kp: Proportional gain
        - ki: Integral gain
        - kd: Derivative gain
        - setpoint: Desired target value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.setpoint = setpoint  # Desired value the controller tries to reach
        self.integral = 0  # Accumulated error for the integral term
        self.previous_error = 0  # Previous error, used to calculate the derivative

    def update(self, current_value, dt):
        """
        Update the PID controller with the current process value and time step.

        Parameters:
        - current_value: The measured value from the system (e.g., motor speed)
        - dt: Time interval since last update (in seconds)

        Returns:
        - control_output: The controller's output (e.g., to send to a motor)
        """
        # Calculate the error between desired setpoint and actual current value
        error = self.setpoint - current_value

        # Integrate the error over time (for the integral term)
        self.integral += error * dt

        # Calculate the rate of change of error (derivative term)
        # Protect against divide-by-zero if dt is 0
        derivative = (error - self.previous_error) / dt if dt > 0 else 0

        # Compute the PID output
        # Each term scales the error differently:
        # - P term reacts to current error
        # - I term reacts to accumulated error
        # - D term predicts future error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Store current error for next derivative calculation
        self.previous_error = error

        return output
