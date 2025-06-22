import numpy as np
import matplotlib.pyplot as plt

def validate_dynamics(sim_data, exp_data, time_vec, plot=False, title=""):
    """
    @param:
        sim_data (np.ndarray): Simulation data array, shapes (n_states, n_samples).
        exp_data (np.ndarray): An array of experimental data with the same shape as sim_data.
        time_vec (np.ndarray): Time vector, shape (n_samples,).
        plot (bool): Whether to draw a contrast curve.
        title (str): Chart title.
    
    @return:
        dict: A dictionary containing maximum error, RMSE, and variance.
    """
    assert sim_data.shape == exp_data.shape, "Simulation and experimental data must have the same shape!"
    
    # Calculate the error index
    errors = exp_data - sim_data
    max_error = np.max(np.abs(errors), axis=1)          # The maximum absolute error of each state quantity
    rmse = np.sqrt(np.mean(errors**2, axis=1))         # RMSE
    variance = np.var(errors, axis=1)                   # error variance
    
    metrics = {
        'max_error': max_error,
        'rmse': rmse,
        'variance': variance,
        'time': time_vec,
        'errors': errors
    }
    
    # Draw comparison curves
    if plot:
        n_states = sim_data.shape[0]
        plt.figure(figsize=(12, 4 * n_states))
        
        for i in range(n_states):
            plt.subplot(n_states, 1, i + 1)
            plt.plot(time_vec, sim_data[i], 'r--', label='Simulation')
            plt.plot(time_vec, exp_data[i], 'b-', label='Experiment')
            plt.fill_between(time_vec, sim_data[i], exp_data[i], color='gray', alpha=0.3, label='Error')
            plt.xlabel('Time (s)')
            plt.ylabel(f'State {i+1}')
            plt.legend()
            plt.title(f'{title} - State {i+1}: Max Error={max_error[i]:.3f}, RMSE={rmse[i]:.3f}')
        
        plt.tight_layout()
        plt.show()
    
    return metrics
