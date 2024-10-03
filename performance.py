# performance.py

import numpy as np

def compute_performance_metrics(desired, actual, time, initial):
    """ Calculates performance metrics for the control system """
    
    # Compute steady-state error (error after stabilization)
    steady_state_error = actual[-1] - desired

    # Find rise time (time to reach 5% of the initial value)
    rise_time_indices = np.where(np.array(actual) <= 0.05 * initial)[0]
    rise_time = time[rise_time_indices[0]] if rise_time_indices.size > 0 else 100000

    # Compute overshoot (if it occurs)
    overshoot = min(actual) - desired if min(actual) < desired else desired - min(actual)

    # Compute settling time (time to remain within 5% of the initial value for at least 0.5 seconds)
    settling_time_threshold = 0.05 * np.abs(initial)  
    settling_time_indices = np.where(np.abs(np.array(actual) - desired) <= settling_time_threshold)[0]

    settling_time = 0
    consecutive_count = 0
    target_time = 0.5  # Target settling time in seconds
    required_samples = int(target_time / (time[1] - time[0]))  # Calculate number of samples required

    for i in range(len(settling_time_indices)):
        # Check if this index is part of the settling time
        if i == 0 or settling_time_indices[i] == settling_time_indices[i - 1] + 1:
            consecutive_count += 1
        else:
            # Reset if not consecutive
            consecutive_count = 1

        # Check if we have enough consecutive samples
        if consecutive_count >= required_samples:
            settling_time = time[settling_time_indices[i]]
            break
        
    if settling_time == 0:
        settling_time = 10000
    return steady_state_error, rise_time, settling_time, overshoot
