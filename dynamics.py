# dynamics.py

import numpy as np
from scipy.integrate import odeint

def quadcopter_dynamics(state, t, u, Ixx, Iyy, Izz):
    """  Implements the rotational dynamics of the quadcopter, following Euler's equations of motion """

    phi, theta, psi, p, q, r = state

    # Torques control inputs for roll, pitch, and yaw
    tau_phi = u[0]
    tau_theta = u[1]
    tau_psi = u[2]

    # Angular accelerations based on torques (Euler's equations of motion)
    p_dot = tau_phi / Ixx
    q_dot = tau_theta / Iyy
    r_dot = tau_psi / Izz

    # Update angular velocities (p, q, r) to update angles (phi, theta, psi)
    phi_dot = p + np.sin(phi) * np.tan(theta) * q + np.cos(phi) * np.tan(theta) * r
    theta_dot = np.cos(phi) * q - np.sin(phi) * r
    psi_dot = (np.sin(phi) / np.cos(theta)) * q + (np.cos(phi) / np.cos(theta)) * r

    return [phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot]


def compute_motor_thrust(control_inputs, total_thrust):
    """ calculates the thrust for each motor of the quadcopter based on the control inputs (roll, pitch, yaw) and the total thrust generated by all motors 
    
    Motor 1 gets more thrust if u_phi and u_theta are both positive.
    Motor 2 gets more thrust if u_phi is negative but u_theta is positive.
    And so on for Motors 3 and 4.
    """
   
    u_phi, u_theta, u_psi = control_inputs
    
    # Distributing thrust among the four motors
    thrusts = np.array([
        total_thrust * (1 + u_phi + u_theta),  # Motor 1
        total_thrust * (1 - u_phi + u_theta),  # Motor 2
        total_thrust * (1 - u_phi - u_theta),  # Motor 3
        total_thrust * (1 + u_phi - u_theta)   # Motor 4
    ])
    
    return thrusts