#include <stdio.h>      // For printf()
#include <math.h>       // For math functions (optional here)
#include <float.h>      // For DBL_MAX (used to track minimum cost)

#define HORIZON 3             //  Number of steps to look into the future
#define CONTROL_OPTIONS 3     //  Number of possible control values: -1, 0, 1
#define TARGET 10.0           //  Goal state we want the system to reach
#define Q 1.0                 //  Penalty weight for error (how far from target)
#define R 0.1                 //  Penalty weight for control effort (to avoid large u)
#define TIME_STEPS 20         //  How many steps to simulate/control

//  Set of discrete control inputs we can apply at each step
const int control_set[CONTROL_OPTIONS] = {-1, 0, 1};

//  Simulates the predicted future state given a control sequence
// and computes the total cost over the prediction horizon.
double simulate_cost(double x_start, int* u_seq) {
    double cost = 0.0;
    double x = x_start;  // Start from the current state

    for (int i = 0; i < HORIZON; i++) {
        int u = u_seq[i];            // Get the control input at step i
        x = x + u;                   // Simple model: x_next = x + u
        double error = x - TARGET;   // Deviation from target
        cost += Q * error * error + R * u * u;  // Total cost = error^2 + effort^2
    }

    return cost;  // Return total cost of this control sequence
}

//  Recursively generates all possible control sequences of length HORIZON
// and keeps track of the one with the lowest cost
void generate_sequences(int depth, int* u_seq, double x, double* best_cost, int* best_u) {
    // Base case: if we've built a full sequence, evaluate its cost
    if (depth == HORIZON) {
        double cost = simulate_cost(x, u_seq);
        if (cost < *best_cost) {
            *best_cost = cost;        // Save lowest cost found
            *best_u = u_seq[0];       // Save the first control input (receding horizon)
        }
        return;
    }

    // Recursive case: try all control values at this step
    for (int i = 0; i < CONTROL_OPTIONS; i++) {
        u_seq[depth] = control_set[i];   // Choose control for current depth
        generate_sequences(depth + 1, u_seq, x, best_cost, best_u);  // Recurse
    }
}

int main() {
    double x = 0.0;               //  Initial state of the system
    int u_seq[HORIZON];           //  Array to hold a sequence of control inputs

    // Run the MPC controller for a number of time steps
    for (int t = 0; t < TIME_STEPS; t++) {
        double best_cost = DBL_MAX;   // Start with "worst" possible cost
        int best_u = 0;               // Placeholder for best first control input

        //  Generate all control sequences and pick the best
        generate_sequences(0, u_seq, x, &best_cost, &best_u);

        //  Apply only the first control input from the best sequence
        x = x + best_u;

        // Print the system state, chosen control, and cost
        printf("Time %2d: x = %5.2f, u = %2d, cost = %6.2f\n", t, x, best_u, best_cost);
    }

    return 0;  // Program completed
}
