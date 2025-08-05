#include <stdio.h>

// System Constants
#define DESIRED_LEVEL 100.0  // Desired Tank Level

int main() {
    double tank_level = 100.0;    // Current level (initially at setpoint)
    double pump_inflow = 0.0;     // Control Variable
    double outflow_disturbance = 10.0;  // Known Outflow Disturbance (e.g., valve outflow)

    // Feedforward Control: set inflow to match outflow disturbance
    pump_inflow = outflow_disturbance;

    // Update Tank Level (simple model: level change = inflow - outflow)
    // For feedforward only, we assume steady-state, so level stays constant
    tank_level += (pump_inflow - outflow_disturbance); // Will be 0 if matched

    printf("Pump Inflow (Control): %.2f units\n", pump_inflow);
    printf("Updated Tank Level: %.2f units\n", tank_level);

    return 0;
}
