# visualization.py

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

def plot_results(time, states, motor_thrusts):
    phi = states[:, 0]
    theta = states[:, 1]
    psi = states[:, 2]

    plt.figure(figsize=(12, 10))

    # Subplot 1: Roll
    plt.subplot(2, 2, 1)
    plt.plot(time, phi, label="Roll (phi)")
    plt.axhline(y=0, color='r', linestyle='--', label="Desired Roll")
    plt.legend(loc='upper right')  
    plt.ylabel("Angle (rad)")
    plt.title("Roll (phi)")
    plt.xlabel("Time (s)")

    # Subplot 2: Pitch
    plt.subplot(2, 2, 2)
    plt.plot(time, theta, label="Pitch (theta)")
    plt.axhline(y=0, color='r', linestyle='--', label="Desired Pitch")
    plt.legend(loc='upper right')  
    plt.ylabel("Angle (rad)")
    plt.title("Pitch (theta)")
    plt.xlabel("Time (s)")

    # Subplot 3: Yaw
    plt.subplot(2, 2, 3)
    plt.plot(time, psi, label="Yaw (psi)")
    plt.axhline(y=0, color='r', linestyle='--', label="Desired Yaw")
    plt.legend(loc='upper right')  
    plt.ylabel("Angle (rad)")
    plt.title("Yaw (psi)")
    plt.xlabel("Time (s)")

    # Subplot 4: Motor Thrusts
    plt.subplot(2, 2, 4)
    plt.plot(time, motor_thrusts[:, 0], label="Motor 1 Thrust")
    plt.plot(time, motor_thrusts[:, 1], label="Motor 2 Thrust")
    plt.plot(time, motor_thrusts[:, 2], label="Motor 3 Thrust")
    plt.plot(time, motor_thrusts[:, 3], label="Motor 4 Thrust")
    plt.ylabel("Thrust (N)")
    plt.xlabel("Time (s)")
    plt.legend(loc='upper right')  
    plt.title("Motor Thrusts")

    plt.subplots_adjust(hspace=5.4)  

    plt.tight_layout()
    plt.savefig("Plots", dpi=150)
    plt.show()

def plot_quadcopter_3d(ax, phi, theta, psi, L, thrusts, current_time):
    arms = np.array([[L, 0, 0], [-L, 0, 0], [0, L, 0], [0, -L, 0]])
    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(phi), -np.sin(phi)],
                       [0, np.sin(phi), np.cos(phi)]])
    R_pitch = np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])
    R_yaw = np.array([[np.cos(psi), -np.sin(psi), 0],
                      [np.sin(psi), np.cos(psi), 0],
                      [0, 0, 1]])

    R = R_yaw @ R_pitch @ R_roll
    rotated_arms = arms @ R.T

    ax.cla()
    ax.plot([rotated_arms[0, 0], rotated_arms[1, 0]],
            [rotated_arms[0, 1], rotated_arms[1, 1]],
            [rotated_arms[0, 2], rotated_arms[1, 2]], 'r', lw=3)  # Arm 1
    ax.plot([rotated_arms[2, 0], rotated_arms[3, 0]],
            [rotated_arms[2, 1], rotated_arms[3, 1]],
            [rotated_arms[2, 2], rotated_arms[3, 2]], 'b', lw=3)  # Arm 2

    # Add circles to represent the motors
    motor_positions = [rotated_arms[i] for i in range(4)]
    for pos in motor_positions:
        ax.scatter(pos[0], pos[1], pos[2], color='orange',
                   s=200, edgecolors='k', label='Motor', alpha=0.7)

    # Add arrows for thrust direction and intensity
    for i, pos in enumerate(motor_positions):
        thrust_vector = np.array([0, 0, thrusts[i]]) / 250
        ax.quiver(pos[0], pos[1], pos[2], thrust_vector[0], thrust_vector[1], thrust_vector[2],
                  color='g', arrow_length_ratio=0.5, linewidth=2, label='Thrust' if i == 0 else "")

    ax.set_xlim([-L, L])
    ax.set_ylim([-L, L])
    ax.set_zlim([-L, L])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f"Visualização Quadcopter 3D (Tempo: {current_time:.2f}s)")
