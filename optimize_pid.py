###############################
######## STILL IN BETA ########
###############################


import numpy as np 
from scipy.integrate import odeint
from scipy.optimize import minimize
from scipy.optimize import differential_evolution
from control import QuadcopterPID, wrap_angle
from dynamics import quadcopter_dynamics, compute_motor_thrust
from config import *
from performance import compute_performance_metrics
from simple_pid import PID

def simulate_quadcopter(roll_pid_params):
    """
    Simulate the quadcopter dynamics with specified roll PID parameters.
    
    Args:
        roll_pid_params (tuple): A tuple containing (Kp, Ki, Kd) for roll PID.
    
    Returns:
        float: The settling time for roll.
    """
    # Unpack the roll PID parameters
    Kp, Ki, Kd = roll_pid_params

    # Round PID parameters to three decimal places
    Kp = round(Kp, 3)
    Ki = round(Ki, 3)
    Kd = round(Kd, 3)

    # Initialize the PID controller with new parameters
    pid_controller = QuadcopterPID()
    pid_controller.pid_roll = PID(Kp=Kp, Ki=Ki, Kd=Kd, setpoint=desired_phi)

    # Time settings for simulation
    time = np.linspace(0, duration, int(duration / timestep))

    # Simulation loop to integrate dynamics and apply PID control
    states = []
    state = initial_state

    for t in time:
        u = pid_controller.control(state)  # Get PID control inputs (torques)
        total_thrust = m * g  # Assume total thrust needed to hover is the mass * gravity
        thrusts = compute_motor_thrust(u, total_thrust)
        state = odeint(quadcopter_dynamics, state, [t, t + timestep], args=(u, Ixx, Iyy, Izz))[1]
        state[2] = wrap_angle(state[2])  # Wrap yaw angle
        states.append(state)

    # Convert states to numpy array for easier manipulation
    states = np.array(states)

    # Compute performance metrics for roll
    roll_metrics = compute_performance_metrics(desired_phi, states[:, 0], time, initial_state[0])

    # Debugging: print the parameters and resulting settling time
    print(f"Testing PID params: Kp={Kp:.4f}, Ki={Ki:.4f}, Kd={Kd:.4f} => Settling time: {roll_metrics[2]:.4f}")

    # Return the settling time
    return roll_metrics[2]  # Settling time is the third metric

def optimize_roll_pid():
    """
    Optimize the roll PID parameters to minimize settling time.
    """
    # Initial guess for the PID parameters
    initial_guess = (2.2, 0.12, 0.115)  # (Kp, Ki, Kd)

    # Bounds for the parameters (optional)
    bounds = [(0.1, 5), (0.01, 1), (0.01, 1)]  # (Kp, Ki, Kd)

    # Perform the optimization
    #result = minimize(simulate_quadcopter, initial_guess, bounds=bounds, method='COBYLA')
    result = minimize(simulate_quadcopter, initial_guess, bounds=bounds, method='Nelder-Mead')
    #result = differential_evolution(simulate_quadcopter, bounds, strategy='best1bin', maxiter=50, popsize=15, tol=0.01)

    # Check the optimization result
    if result.success:
        optimized_params = result.x
        # Round the optimized parameters to three decimal places
        optimized_params = np.round(optimized_params, 3)
        print(f"Optimized Roll PID Parameters: Kp={optimized_params[0]:.4f}, Ki={optimized_params[1]:.4f}, Kd={optimized_params[2]:.4f}")
        print(f"Minimum Settling Time: {result.fun:.4f}s")
    else:
        print("Optimization failed:", result.message)

if __name__ == "__main__":
    optimize_roll_pid()
