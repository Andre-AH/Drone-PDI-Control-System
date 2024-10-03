# main.py

# References:
# Borase, Rakesh P.; Maghade, D. K.; Sondkar, S. Y.; Pawar, S. N. . (2020). A review of PID control, tuning methods and applications. International Journal of Dynamics and Control, doi:10.1007/s40435-020-00665-4 
# https://www.isa.org/intech-home/2023/june-2023/features/fundamentals-pid-control
# https://medium.com/@sayedebad.777/drones-and-pid-control-an-introductory-guide-to-aerial-robotics-9cf24ffb1853#:~:text=PID%20controllers%20are%20used%20to%20maintain%20and%20control%20the%20drone%E2%80%99s
# https://medium.com/@squonk-/understanding-pid-controllers-stable-flight-in-drones-and-beyond-861b1471c026#:~:text=The%20PID%20(Proportional-Integral-Derivative)%20control%20system%20combines%20proportional,%20derivative,%20and


import numpy as np
from scipy.integrate import odeint
from dynamics import quadcopter_dynamics, compute_motor_thrust
from control import QuadcopterPID, wrap_angle
from visualization import plot_results, plot_quadcopter_3d
from performance import compute_performance_metrics
from config import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime 

# Time settings for simulation
time = np.linspace(0, duration, int(duration / timestep))

# Initialize PID controller
pid_controller = QuadcopterPID()

# Simulation loop to integrate dynamics and apply PID control
states = []
motor_thrusts = []
state = initial_state

for t in time:
    u = pid_controller.control(state)  # Get PID control inputs (torques)
    total_thrust = m * g  # Assume total thrust needed to hover is the mass * gravity
    thrusts = compute_motor_thrust(
        u, total_thrust)  # Compute each motor thrust
    motor_thrusts.append(thrusts)
    state = odeint(quadcopter_dynamics, state, [
                   t, t + timestep], args=(u, Ixx, Iyy, Izz))[1]

    # Wrap yaw angle after each integration step
    state[2] = wrap_angle(state[2])
    states.append(state)

# Convert states and thrusts to numpy array for easier manipulation
states = np.array(states)
motor_thrusts = np.array(motor_thrusts)

# Plot results
plot_results(time, states, motor_thrusts)

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')


 
def update(i):
    """ Animation update function """
    current_time = time[i]  # Get the current time from the time array
    plot_quadcopter_3d(ax, states[i, 0], states[i, 1],
                       states[i, 2], L, motor_thrusts[i], current_time)


# Create the animation
interval_ms = 10 
ani = animation.FuncAnimation(
    fig, update, frames=len(time), interval=interval_ms)

# Save the animation as a GIF 
# ani.save('quadcopter_rotation.gif', writer='pillow')

plt.show()

# Compute performance metrics for roll, pitch, and yaw
roll_metrics = compute_performance_metrics(
    desired_phi, states[:, 0], time, initial_state[0])
pitch_metrics = compute_performance_metrics(
    desired_theta, states[:, 1], time, initial_state[1])
yaw_metrics = compute_performance_metrics(
    desired_psi, states[:, 2], time, initial_state[2])


# Prepare initial conditions and parameters for logging
initial_conditions = (
    "-------------------------------------------------------------------------\n"
    f"Initial Conditions and Parameters:\n"
    "-------------------------------------------------------------------------\n"
    f"Mass (m): {m:.2f} kg\n"
    f"Moments of Inertia: Ixx={Ixx:.3f}, Iyy={Iyy:.3f}, Izz={Izz:.3f} kg*m^2\n"
    f"Gravitational Constant (g): {g:.2f} m/s^2\n"
    f"Arm Length (L): {L:.2f} m\n"
    f"Yaw Control Constant (k): {k:.2f}\n"
    f"Initial State (angles and angular velocities): {initial_state} \n"
    f"Simulation Parameters:\n"
    f"Timestep: {timestep:.6f} s\n"
    f"Duration: {duration:.1f} s\n\n"
)

pid_params = (
    "-------------------------------------------------------------------------\n"
    f"PID Parameters:\n"
    "-------------------------------------------------------------------------\n"
    f"Roll: Kp={pid_controller.pid_roll.Kp}, Ki={pid_controller.pid_roll.Ki}, Kd={pid_controller.pid_roll.Kd}, "
    f"Output Limits={pid_controller.pid_roll.output_limits}\n"
    f"Pitch: Kp={pid_controller.pid_pitch.Kp}, Ki={pid_controller.pid_pitch.Ki}, Kd={pid_controller.pid_pitch.Kd}, "
    f"Output Limits={pid_controller.pid_pitch.output_limits}\n"
    f"Yaw: Kp={pid_controller.pid_yaw.Kp}, Ki={pid_controller.pid_yaw.Ki}, Kd={pid_controller.pid_yaw.Kd}, "
    f"Output Limits={pid_controller.pid_yaw.output_limits}\n\n"
)

# Prepare performance data for logging
performance_data = [
    "-------------------------------------------------------------------------\n"
    f"Performance Parameters:\n"
    "-------------------------------------------------------------------------\n"
    f"Performance Metrics for Roll (phi):\n"
    f"Steady-State Error: {roll_metrics[0]:.4f}, Rise Time: {roll_metrics[1]:.4f}s, "
    f"Settling Time: {roll_metrics[2]:.4f}s, Overshoot: {roll_metrics[3]:.4f}\n",

    f"Performance Metrics for Pitch (theta):\n"
    f"Steady-State Error: {pitch_metrics[0]:.4f}, Rise Time: {pitch_metrics[1]:.4f}s, "
    f"Settling Time: {pitch_metrics[2]:.4f}s, Overshoot: {pitch_metrics[3]:.4f}\n",

    f"Performance Metrics for Yaw (psi):\n"
    f"Steady-State Error: {yaw_metrics[0]:.4f}, Rise Time: {yaw_metrics[1]:.4f}s, "
    f"Settling Time: {yaw_metrics[2]:.4f}s, Overshoot: {yaw_metrics[3]:.4f}\n \n"
]

# Print results
#for data in performance_data:
    #print(data)

current_datetime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

# Save performance metrics to log.txt
with open('log.txt', 'a') as log_file:
    log_file.write(f"Log Entry: {current_datetime}\n")
    log_file.write(initial_conditions)
    log_file.write(pid_params)
    log_file.writelines(performance_data)
