import time
from controller import RobotVelocityController
import numpy as np
from tutorial3 import *

def parabolic_blend_coeffs(start, end, duration, t_b = None, acceleration = None):
    """
    Compute parabolic blend coefficients for trajectory planning.
    
    Functionality:
        Calculates the coefficients for a parabolic blend trajectory that smoothly
        transitions between two linear segments. The blend occurs over a specified
        time t_b, and the total duration of the trajectory is given by duration.
    
    Input:
        start (np.ndarray): Starting joint positions (6 dimensions)
        end (np.ndarray): Ending joint positions (6 dimensions)
        duration (float): Total duration of the trajectory (seconds)
        t_b (np.ndarray or None): Blend time for acceleration and deceleration phases (seconds)
        acceleration (np.ndarray or None): Acceleration magnitude for the blend phase
    
    Output:
        t_b (np.ndarray or None): Computed blend time for each joint
        acceleration (np.ndarray or None): Computed acceleration for each joint
        Returns (None, None) if parameters are invalid
    """
    if t_b is not None:
        if np.any(t_b >= duration / 2):
            print("Blend time t_b must be less than half of the total duration.")
            return None, None
        
        # Calculate the acceleration based on the provided blend time
        acceleration = 

    elif acceleration is not None:
        if np.any(duration**2 * acceleration**2 - 4*(end - start)*acceleration < 0):
            print("Acceleration is too low for the given duration and distance.")
            return None, None
        # Calculate the blend time based on the provided acceleration
        t_b = 

    else:
        print("Either t_b or acceleration must be provided.")
        return None, None
    return t_b, acceleration

def parabolic_blend_trajectory(start, end, t, duration, t_b = None, acceleration = None):
    """
    Generate a parabolic blend trajectory from start to end over time duration.
    
    Functionality:
        Computes the position and velocity for a parabolic blend trajectory at time t.
        The trajectory consists of three phases: acceleration, constant velocity,
        and deceleration.

    Input:
        start (np.ndarray): Starting joint positions (6 dimensions)
        end (np.ndarray): Ending joint positions (6 dimensions)
        t (float): Current time elapsed since trajectory start (seconds)
        duration (float): Total duration of the trajectory (seconds)
        t_b (np.ndarray or None): Blend time for acceleration and deceleration phases (seconds)
        acceleration (np.ndarray or None): Acceleration magnitude for the blend phase (if t_b is not provided)
    
    Output:
        target_pos (np.ndarray): Target joint positions at time t (6 dimensions)
        target_speed (np.ndarray): Target joint velocities at time t (6 dimensions)
    """
    t_b, acceleration = parabolic_blend_coeffs(start, end, duration, t_b, acceleration)
    if t_b is None or acceleration is None:
        print("Failed to compute parabolic blend coefficients.")
        return np.zeros_like(start), np.zeros_like(start)

    target_pos = np.zeros_like(start)
    target_speed = np.zeros_like(start)
    for i in range(6):
        # Compute position and velocity for each joint,
        # based on the current time t and the blend parameters

    return target_pos, target_speed

def execute_trajectory(controller, point_list, duration_list, traj_type = 'cubic', interm_type="average_velocity", acceleration=None, t_b=None):
    """
    Execute a trajectory using the given robot controller.
    
    Functionality:
        Executes either a linear, cubic, or parabolic blend trajectory by continuously computing target
        positions and velocities, sending velocity commands to the robot, and visualizing
        the trajectory in real-time.
    
    Input:
        controller (RobotVelocityController): Robot controller instance for sending commands
        point_list (list of np.ndarray): List of joint positions in radians (6 dimensions each)
        duration_list (list of float): List of durations for each segment (seconds)
        traj_type (str): Type of trajectory - 'linear', 'cubic', or 'parabolic_blend' (default: 'cubic')
        interm_type (str): Type of intermediate velocity calculation for cubic trajectories 
                          - 'average_velocity' or other types (default: 'average_velocity')
        acceleration (np.ndarray or None): Acceleration values for parabolic blend trajectory (default: None)
        t_b (np.ndarray or None): Blend times for parabolic blend trajectory (default: None)
    
    Output:
        None (sends commands to robot and updates visualization)
    """
    # Precompute cubic coefficients
    if traj_type == 'cubic':
        coeffs_list = multiple_cubic_coeffs(point_list, duration_list, interm_type=interm_type)
    
    target_speed = [0,0,0,0,0,0]
    target_pos = point_list[0]
    t = 0
    controller.draw_trajectory(target_speed, target_pos, t)
    
    start_t = time.time()
    try:
        while time.time() - start_t < duration_list[-1]:
            t = time.time() - start_t
            stage = 0
            stage_t = t
            stage_duration = duration_list[0]
            
            # Determine current trajectory stage
            for i in range(len(duration_list)):
                if t < duration_list[i]:
                    stage = i
                    break
            if stage > 0:
                stage_t -= duration_list[stage-1]
                stage_duration = duration_list[stage] - duration_list[stage-1]

            # Compute target position and velocity based on trajectory type
            if traj_type == 'linear':
                target_pos, target_speed = linear_trajectory(point_list[stage], point_list[stage+1], stage_t, stage_duration)
            elif traj_type == 'cubic':
                cubic_coeffs = coeffs_list[stage]
                target_pos, target_speed = cubic_trajectory(cubic_coeffs, stage_t)
            elif traj_type == 'parabolic_blend':
                target_pos, target_speed = parabolic_blend_trajectory(point_list[stage], point_list[stage+1], stage_t, stage_duration, t_b=t_b, acceleration=acceleration)
            
            # Send commands to robot and update visualization
            controller.draw_trajectory(target_speed, target_pos, t)
            controller.speedJ(target_speed)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
    print("Elapsed time:", time.time() - start_t)

if __name__ == "__main__":
    # Initialize robot controller
    controller = RobotVelocityController()
    controller.start()

    # Define waypoints
    P0 = np.array([0, -90, 0, -90, 0, 0])
    P1 = np.array([90, -60, 60, -90, -90, 0])

    # First trajectory: P0 to P1 with specified acceleration
    point_list = [np.deg2rad(P0), np.deg2rad(P1)]
    duration_list = [10.0]

    acceleration = np.array([0.1, 0.1, 0.1, 0.1, -0.1, 0.1])
    t_b = None

    execute_trajectory(controller, point_list=point_list, duration_list=duration_list, traj_type="parabolic_blend", acceleration=acceleration, t_b=t_b)

    # Second trajectory: P1 to P0 with specified blend time
    point_list = [np.deg2rad(P1), np.deg2rad(P0)]
    duration_list = [10.0]

    acceleration = None
    t_b = np.array([2.0] * 6)

    execute_trajectory(controller, point_list=point_list, duration_list=duration_list, traj_type="parabolic_blend", acceleration=acceleration, t_b=t_b)
    
    # Stop the robot
    controller.stop()

    # Keep the plot window open
    controller.keep_plot_open()

    # Close the connection
    controller.close()