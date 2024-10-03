# Quadcopter PID Controller Simulation

This project simulates a quadcopter's flight dynamics using a PID (Proportional-Integral-Derivative) controller. The program computes the quadcopter’s behavior in terms of roll, pitch, and yaw stability based on a simulated PID control system, using numerical integration for dynamics and visualizations for understanding the quadcopter's motion.

## Table of Contents

- [Introduction]
- [Features]
- [Usage]
- [Files]
- [References]

## Introduction

PID controllers are widely used for stabilizing drones by controlling their roll, pitch, and yaw. In this project, a simulation of a quadcopter using a PID controllers is implemented in Python. The program integrates the drone's dynamics over time and applies the PID controller to achieve stability and maintain the desired roll, pitch, and yaw.

## Features

- **PID Control**: Three independent PID controllers (for roll, pitch, and yaw) to stabilize the quadcopter.
- **Dynamic Simulation**: Uses the drone's physical parameters and integrates motion over time to simulate realistic behavior.
- **3D Visualization**: A 3D animation of the quadcopter’s movements over time.
- **Performance Metrics**: Calculation of key performance metrics like steady-state error, rise time, settling time, and overshoot.
- **Logging**: Logs the simulation data, initial conditions, and PID parameters with timestamps for tracking multiple simulation runs.

## Usage

To run the simulation, execute the `main.py` file:

```bash
python main.py
```

The parameters are changed on `config.py` and `control.py`

## Files

- **`main.py`**: Main script that runs the simulation, applies the PID controller, and generates visualizations.
- **`dynamics.py`**: Contains the dynamics equations of the quadcopter (e.g., angular velocity, motor thrust computation).
- **`control.py`**: Implements the PID control algorithm.
- **`visualization.py`**: Generates the visualizations of the quadcopter’s movement.
- **`performance.py`**: Calculates performance metrics for the system.
- **`config.py`**: Defines initial parameters for the simulation.

## References

- Borase, Rakesh P.; Maghade, D. K.; Sondkar, S. Y.; Pawar, S. N. (2020). A review of PID control, tuning methods, and applications. _International Journal of Dynamics and Control_, doi:10.1007/s40435-020-00665-4
- ISA - _Fundamentals of PID Control_ (June 2023). [Link](https://www.isa.org/intech-home/2023/june-2023/features/fundamentals-pid-control)
- Medium - _Drones and PID Control: An Introductory Guide to Aerial Robotics_ [Link](https://medium.com/@sayedebad.777/drones-and-pid-control-an-introductory-guide-to-aerial-robotics-9cf24ffb1853)
- Medium - _Understanding PID Controllers: Stable Flight in Drones and Beyond_ [Link](https://medium.com/@squonk-/understanding-pid-controllers-stable-flight-in-drones-and-beyond-861b1471c026)
