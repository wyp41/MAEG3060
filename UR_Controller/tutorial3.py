import time
from controller import RobotVelocityController
import numpy as np

# Please implement zero_velocity and average_velocity functions here

def zero_velocity(point_list, duration_list, joint_num = 6):
    """
    Generate zero velocities at all waypoints for trajectory planning.
    
    Functionality:
        Creates a list of zero velocity vectors for each waypoint, ensuring the robot
        comes to a complete stop at each intermediate position. This results in a
        stop-and-go motion pattern.
    
    Input:
        point_list (list of np.ndarray): List of joint positions (waypoints) in radians
        duration_list (list of float): List of durations for each segment (seconds)
        joint_num (int): Number of joints (default: 6)
    
    Output:
        speed_list (list of np.ndarray): List of zero velocity vectors at each waypoint (joint_num dimensions each)
    """
    
    return speed_list

def average_velocity(point_list, duration_list, joint_num = 6):
    """
    Compute average velocities at intermediate waypoints for smooth trajectory transitions.
    
    Functionality:
        Calculates intermediate velocities at each waypoint by averaging the velocities
        from adjacent segments. If velocity directions differ (product <= 0), sets
        velocity to zero to avoid oscillations. Uses zero velocity at start and end points.
    
    Input:
        point_list (list of np.ndarray): List of joint positions (waypoints) in radians
        duration_list (list of float): List of durations for each segment (seconds)
        joint_num (int): Number of joints (default: 6)
    
    Output:
        speed_list (list of np.ndarray): List of velocity vectors at each waypoint (joint_num dimensions each)
    """
    N = len(point_list)

    # Initialize speed list with zero velocity at the starting point
    speed_list = [np.zeros(joint_num)]
    
    # Iterate through intermediate waypoints (excluding start and end)
    for i in range(1, N-1):
        # Get three consecutive waypoints for velocity calculation
        point1 = point_list[i-1]
        point2 = point_list[i]
        point3 = point_list[i+1]
        duration1 = duration_list[i-1] - duration_list[i-2] if i > 1 else duration_list[i-1]
        duration2 = duration_list[i] - duration_list[i-1]

        # Calculate velocities for the two adjacent segments
        # Check if velocities have the same direction (positive product means same direction)
        # Average the velocities if they have the same direction; otherwise, set to zero
        # mid_speed should be a 6 dimension vector
        
        speed_list.append(np.array(mid_speed))

    # Add zero velocity at the ending point
    speed_list.append(np.zeros(joint_num))
    return speed_list

def compute_cubic_coeffs(start, end, start_speed, end_speed, duration):
    """
    Compute cubic polynomial coefficients for trajectory planning.
    
    Functionality:
        Solves for the coefficients of a cubic polynomial that connects start and end
        positions with specified velocities at both endpoints. The polynomial is of the form:
        q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    
    Input:
        start (np.ndarray): Starting joint positions (6 dimensions)
        end (np.ndarray): Ending joint positions (6 dimensions)
        start_speed (np.ndarray): Starting joint velocities (6 dimensions)
        end_speed (np.ndarray): Ending joint velocities (6 dimensions)
        duration (float): Total duration of the trajectory (seconds)
    
    Output:
        coeffs (np.ndarray): Polynomial coefficients [a0, a1, a2, a3] for each joint (4x6 matrix)
    """
    Time_related_matrix = np.array([[1, 0, 0, 0],
                                    [1, duration, duration**2, duration**3],
                                    [0, 1, 0, 0],
                                    [0, 1, 2*duration, 3*duration**2]])
    
    # targets are [position_start, position_end, velocity_start, velocity_end]
    # Build a (4, N) matrix so each joint (N) gets its own cubic polynomial.
    targets = np.vstack([start, end, start_speed, end_speed])

    # solve the coefficients for the cubic polynomial
    coeffs = np.linalg.solve(Time_related_matrix, targets)
    return coeffs

def multiple_cubic_coeffs(point_list, duration_list, interm_type="average_velocity"):
    """
    Compute cubic polynomial coefficients for multi-segment trajectory planning.
    
    Functionality:
        Generates cubic polynomial coefficients for each segment of a multi-waypoint
        trajectory. Intermediate velocities are computed based on the specified type
        (average_velocity for smooth transitions or zero_velocity for stop-and-go motion).
        Each segment's duration is calculated from the cumulative duration list.
    
    Input:
        point_list (list of np.ndarray): List of joint positions (waypoints) in radians (6 dimensions each)
        duration_list (list of float): List of cumulative durations at each waypoint (seconds)
        interm_type (str): Type of intermediate velocity calculation - 'average_velocity' or 'zero_velocity' (default: 'average_velocity')
    
    Output:
        coeffs_list (list of np.ndarray): List of polynomial coefficients for each segment (each is 4x6 matrix)
    """
    N = len(point_list)
    if interm_type == "average_velocity":
        speed_list = average_velocity(point_list, duration_list)
    elif interm_type == "zero_velocity":
        speed_list = zero_velocity(point_list, duration_list)
        
    coeffs_list = []
    for i in range(N-1):
        start = point_list[i]
        end = point_list[i+1]
        start_speed = speed_list[i]
        end_speed = speed_list[i+1]
        if i > 0:
            duration = duration_list[i] - duration_list[i-1]
        else:
            duration = duration_list[i]
        coeffs = compute_cubic_coeffs(start, end, start_speed, end_speed, duration)
        coeffs_list.append(coeffs)

    return coeffs_list

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

    target_pos = coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3
    target_speed = coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2
    
    return target_pos, target_speed

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
    target_pos = start + (end - start) * (t / duration)
    target_speed = (end - start) / duration
    return target_pos,target_speed

def execute_trajectory(controller, point_list, duration_list, traj_type = 'cubic', interm_type="average_velocity"):
    """
    Execute a trajectory using the given robot controller.
    
    Functionality:
        Executes either a linear or cubic trajectory by continuously computing target
        positions and velocities, sending velocity commands to the robot, and visualizing
        the trajectory in real-time.
    
    Input:
        controller (RobotVelocityController): Robot controller instance for sending commands
        point_list (list of np.ndarray): List of joint positions in radians (6 dimensions each)
        duration_list (list of float): List of durations for each segment (seconds)
        traj_type (str): Type of trajectory - 'linear' or 'cubic' (default: 'cubic')
    
    Output:
        None (sends commands to robot and updates visualization)
    """
    # Precompute cubic coefficients
    if traj_type == 'cubic':
        coeffs_list = multiple_cubic_coeffs(point_list, duration_list, interm_type=interm_type)
    
    start_t = time.time()
    try:
        while time.time() - start_t < duration_list[-1]:
            t = time.time() - start_t
            stage = 0
            stage_t = t
            stage_duration = duration_list[0]
            for i in range(len(duration_list)):
                if t < duration_list[i]:
                    stage = i
                    break
            if stage > 0:
                stage_t -= duration_list[stage-1]
                stage_duration = duration_list[stage] - duration_list[stage-1]

            if traj_type == 'linear':
                target_pos, target_speed = linear_trajectory(point_list[stage], point_list[stage+1], stage_t, stage_duration)
            elif traj_type == 'cubic':
                cubic_coeffs = coeffs_list[stage]
                target_pos, target_speed = cubic_trajectory(cubic_coeffs, stage_t)
            
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
    P2 = np.array([120, -90, 90, -45, -90, 0])

    point_list = [np.deg2rad(P0), np.deg2rad(P1), np.deg2rad(P2), np.deg2rad(P1), np.deg2rad(P0)]
    duration_list = [10.0, 20.0, 30.0, 40.0]

    execute_trajectory(controller, point_list=point_list, duration_list=duration_list, traj_type="cubic", interm_type="zero_velocity")
    
    # Stop the robot
    controller.stop()

    # Keep the plot window open
    controller.keep_plot_open()

    # Close the connection
    controller.close()