# control.py

import numpy as np
from simple_pid import PID
from config import desired_phi, desired_theta, desired_psi

class QuadcopterPID:
    def __init__(self):
        # PID parameters:
            # Kp -> Proportional constant
            # Ki -> Integral constant
            # Kd -> Derivative constant
        self.pid_roll = PID(Kp=1.7, Ki=0.12, Kd=0.115, setpoint=desired_phi)
        self.pid_pitch = PID(Kp=1.7, Ki=0.12, Kd=0.115, setpoint=desired_theta)
        self.pid_yaw = PID(Kp=1.7, Ki=0.12, Kd=0.115, setpoint=desired_psi)

        # Limit the output of the PID to prevent excessive torques
        self.pid_roll.output_limits = (-5, 5)
        self.pid_pitch.output_limits = (-5, 5)
        self.pid_yaw.output_limits = (-3, 3)

    def control(self, state):
        """ calculate the control outputs based on the current state """
        phi, theta, psi, _, _, _ = state
        u_phi = self.pid_roll(phi)
        u_theta = self.pid_pitch(theta)
        u_psi = self.pid_yaw(wrap_angle(psi))
        return [u_phi, u_theta, u_psi]

def wrap_angle(angle):
    """Wrap yaw angle to keep it within [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi
