# config.py

# Quadcopter parameters
Ixx, Iyy, Izz = 0.014, 0.018, 0.022  # Moments of inertia for roll, pitch, yaw
m = 1.5  # Mass of quadcopter in kg
g = 9.81  # Gravitational constant (m/s^2)
L = 0.25  # Distance from center to each motor (arm length)
k = 0.01  # Constant for yaw control (not strict)

# Initial state [phi, theta, psi, p, q, r] (inictial angles and angular velocities in radians)
initial_state = [0.5, 0.7, 0.0, 0, 0.05, 0]

# Simulation parameters
timestep = 0.0002 
duration = 2.0

# Desired angles (setpoints)
desired_phi = 0  # Roll (rad)
desired_theta = 0  # Pitch (rad)
desired_psi = 0  # Yaw (rad)
