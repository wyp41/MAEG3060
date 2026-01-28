import time
from controller import RobotVelocityController
import numpy as np

def linear_trajectory(start, end, t, duration):
    """
    Generate a linear trajectory from start to end over time duration.
    
    Functionality:
        Computes the position and velocity for a linear trajectory at time t.
        The trajectory moves at constant velocity from start to end position.
    
    Input:
        start (np.ndarray): Starting joint positions (6 dimensions)
        end (np.ndarray): Ending joint positions (6 dimensions)
        t (float): Current time elapsed since trajectory start (seconds)
        duration (float): Total duration of the trajectory (seconds)
    
    Output:
        target_pos (np.ndarray): Target joint positions at time t (6 dimensions)
        target_speed (np.ndarray): Target joint velocities at time t (6 dimensions)
    """
    
    # Please implement the function here

    return target_pos,target_speed

def compute_cubic_coeffs(start, end, duration):
    """
    Compute cubic polynomial coefficients for trajectory planning.
    
    Functionality:
        Solves for the coefficients of a cubic polynomial that connects start and end
        positions with zero velocity at both endpoints. The polynomial is of the form:
        q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    
    Input:
        start (np.ndarray): Starting joint positions (6 dimensions)
        end (np.ndarray): Ending joint positions (6 dimensions)
        duration (float): Total duration of the trajectory (seconds)
    
    Output:
        coeffs (np.ndarray): Polynomial coefficients [a0, a1, a2, a3] for each joint (4x6 matrix)
    """
    
    # Please implement the function here

    return coeffs

def cubic_trajectory(coeffs, t):
    """
    Generate a cubic trajectory using precomputed coefficients.
    
    Functionality:
        Evaluates the cubic polynomial and its derivative at time t to get
        the target position and velocity. Uses precomputed coefficients for efficiency.
    
    Input:
        coeffs (np.ndarray): Polynomial coefficients [a0, a1, a2, a3] for each joint (4x6 matrix)
        t (float): Current time elapsed since trajectory start (seconds)
    
    Output:
        target_pos (np.ndarray): Target joint positions at time t (6 dimensions)
        target_speed (np.ndarray): Target joint velocities at time t (6 dimensions)
    """

    # Please implement the function here
    
    return target_pos, target_speed

def execute_trajectory(controller, start, end, duration, traj_type = 'linear'):
    """
    Execute a trajectory using the given robot controller.
    
    Functionality:
        Executes either a linear or cubic trajectory by continuously computing target
        positions and velocities, sending velocity commands to the robot, and visualizing
        the trajectory in real-time.
    
    Input:
        controller (RobotVelocityController): Robot controller instance for sending commands
        start (np.ndarray): Starting joint positions in radians (6 dimensions)
        end (np.ndarray): Ending joint positions in radians (6 dimensions)
        duration (float): Total duration of the trajectory (seconds)
        traj_type (str): Type of trajectory - 'linear' or 'cubic' (default: 'linear')
    
    Output:
        None (sends commands to robot and updates visualization)
    """
    # Precompute cubic coefficients if needed
    if traj_type == 'cubic':
        cubic_coeffs = compute_cubic_coeffs(start, end, duration)
    
    start_t = time.time()
    try:
        while time.time() - start_t < duration:
            t = time.time() - start_t
            if traj_type == 'linear':
                target_pos, target_speed = linear_trajectory(start, end, t, duration)
            elif traj_type == 'cubic':
                target_pos, target_speed = cubic_trajectory(cubic_coeffs, t)
            
            controller.draw_trajectory(target_speed, target_pos, t)
            controller.speedJ(target_speed)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
    print("Elapsed time:", time.time() - start_t)

if __name__ == "__main__":
    controller = RobotVelocityController()
    controller.start()

    P0 = np.array([0, -90, 0, -90, 0, 0])
    P1 = np.array([90, -60, 60, -90, -90, 0])

    start = np.deg2rad(P0)
    end = np.deg2rad(P1)

    execute_trajectory(controller, start=start, end=end, duration=20.0, traj_type='cubic')

    time.sleep(5)

    start = np.deg2rad(P1)
    end = np.deg2rad(P0)

    execute_trajectory(controller, start=start, end=end, duration=20.0, traj_type='cubic')
    
    # Stop the robot
    controller.stop()

    # Keep the plot window open
    controller.keep_plot_open()

    # Close the connection
    controller.close()