import time
from controller import RobotVelocityController
import numpy as np
from tutorial3 import cubic_trajectory, multiple_cubic_coeffs

def rotation_matrix_to_angle_axis(R):
    """
        Convert a 3x3 rotation matrix to a 3D angle-axis vector.

        Args:
                R (np.ndarray): Rotation matrix of shape (3, 3).

        Returns:
                np.ndarray: Angle-axis vector of shape (3,).
    """
    # Extract angle from trace: trace(R) = 1 + 2*cos(angle)
    cos_angle = (np.trace(R) - 1) / 2
    cos_angle = np.clip(cos_angle, -1.0, 1.0)  # Clamp to valid range for arccos
    angle = np.arccos(cos_angle)
    
    # Handle near-zero rotations to avoid numerical instability
    if angle < 1e-6:
        return np.zeros(3)
    
    # Extract rotation axis from skew-symmetric part of rotation matrix
    # Uses the property: R - R^T = 2*sin(angle)*[axis]_x
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2 * np.sin(angle))
    
    # Return angle-axis: magnitude = angle, direction = axis
    return angle * axis

def angle_axis_to_rotation_matrix(angle_axis):
    """
    Convert a 3D angle-axis vector to a 3x3 rotation matrix.

    The input vector encodes axis and angle together:
    - angle = ||angle_axis||
    - axis = angle_axis / angle

    Args:
        angle_axis (np.ndarray): Angle-axis vector of shape (3,).

    Returns:
        np.ndarray: Rotation matrix of shape (3, 3).

    Notes:
        If the angle magnitude is very small, this function returns identity
        to avoid division by nearly zero values.
    """
    angle = np.linalg.norm(angle_axis)
    
    if angle < 1e-6:
        return np.eye(3)
    
    axis = angle_axis / angle
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    
    return R

def pointing_vector_to_angle_axis(pointing_vector1, pointing_vector2, x_direction1, x_direction2):
    """
    Compute relative orientation change as angle-axis from two frame definitions.

    Each frame is defined by:
    - A pointing vector used as local z-axis
    - An x-direction reference used to build local x-axis after orthogonalization

    The relative rotation from frame 1 to frame 2 is then:
        R_rel = R2 * R1^T
    and returned as an angle-axis vector.

    Args:
        pointing_vector1 (np.ndarray): Frame-1 z-axis, shape (3,).
        pointing_vector2 (np.ndarray): Frame-2 z-axis, shape (3,).
        x_direction1 (np.ndarray): Frame-1 x-axis, shape (3,).
        x_direction2 (np.ndarray): Frame-2 x-axis, shape (3,).

    Returns:
        np.ndarray: Relative angle-axis vector from frame 1 to frame 2.
    """
    # Normalize z-axes (pointing vectors)
    z_axis1 = pointing_vector1 / np.linalg.norm(pointing_vector1)
    z_axis2 = pointing_vector2 / np.linalg.norm(pointing_vector2)
    
    # Construct x-axes by orthogonalizing x_direction with respect to z_axis
    x_temp1 = x_direction1 - np.dot(x_direction1, z_axis1) * z_axis1
    x_axis1 = x_temp1 / np.linalg.norm(x_temp1)
    
    x_temp2 = x_direction2 - np.dot(x_direction2, z_axis2) * z_axis2
    x_axis2 = x_temp2 / np.linalg.norm(x_temp2)
    
    # y-axes are cross products of z and x
    y_axis1 = np.cross(z_axis1, x_axis1)
    y_axis2 = np.cross(z_axis2, x_axis2)
    
    # Construct rotation matrices [x, y, z] as columns
    R1 = np.column_stack((x_axis1, y_axis1, z_axis1))
    R2 = np.column_stack((x_axis2, y_axis2, z_axis2))
    
    # Calculate relative rotation from frame 1 to frame 2
    R_rel = R2 @ R1.T
    
    # Convert rotation matrix to angle-axis representation
    angle_axis = rotation_matrix_to_angle_axis(R_rel)
    
    return angle_axis

def execute_trajectory(controller, point_list, pointing_vectors, x_directions, duration_list):
    """
    Execute Cartesian and orientation trajectory tracking with velocity commands.

    Trajectory state convention in this script:
    - point_list columns: [x, y, z, s]
      where s is a scalar progress variable used for orientation interpolation.
    - target_speed from cubic_trajectory: [vx, vy, vz, ds/dt]
    - Angular velocity is computed as:
      w_desired = (ds/dt) * delta_angle_axis

    Args:
        controller (RobotVelocityController): Active robot velocity controller.
        point_list (np.ndarray): Waypoints of shape (N, 4), containing
            Cartesian position in meters and scalar progress s.
        pointing_vectors (np.ndarray): Orientation z-axis hints for each waypoint,
            shape (N, 3).
        x_directions (np.ndarray): Orientation x-axis hints for each waypoint,
            shape (N, 3).
        duration_list (list[float]): Cumulative segment end times in seconds,
            length N-1.
    """
    # Calculate transformation in angle-axis format for each segment
    angle_axis_list = []

    # Compute rotation transformations between consecutive waypoint orientations
    for i in range(len(pointing_vectors)-1):
        delta_angle_axis = pointing_vector_to_angle_axis(
            pointing_vectors[i], pointing_vectors[i+1],
            x_directions[i], x_directions[i+1]
        )      
        angle_axis_list.append(delta_angle_axis)
    
    # Generate cubic trajectory coefficients for all segments
    coeffs_list = multiple_cubic_coeffs(point_list, duration_list, interm_type="zero_velocity")

    # Initialize 3D visualization with initial waypoints and orientations
    controller.init_3d_visualization(point_list, pointing_vectors, x_directions, draw_coordinate=True)
    
    # Track last visualization update time
    last_plot_t = 0.0

    # Record start time and execute trajectory loop
    start_t = time.time()
    try:
        while time.time() - start_t < duration_list[-1]:
            # Get elapsed time since trajectory start
            t = time.time() - start_t
            stage = 0
            stage_t = t
            stage_duration = duration_list[0]
            
            # Determine which trajectory segment we are currently in
            for i in range(len(duration_list)):
                if t < duration_list[i]:
                    stage = i
                    break
            
            # Calculate time within current segment
            if stage > 0:
                stage_t -= duration_list[stage-1]
                stage_duration = duration_list[stage] - duration_list[stage-1]

            # Get cubic trajectory and velocity at current time
            cubic_coeffs = coeffs_list[stage]
            target_pos, target_speed = cubic_trajectory(cubic_coeffs, stage_t)

            # Compute desired angular velocity from rotation rate and angle-axis representation
            w_desired = target_speed[3] * angle_axis_list[stage]

            # Extract desired linear velocity from trajectory
            v_desired = target_speed[:3]

            # Combine linear and angular velocities
            total_v_desired = np.hstack((v_desired, w_desired))

            # Estimate TCP pose based on velocities
            p_est, R_est = controller.tcp_pose_estimate(v_desired, w_desired, t)

            # Send velocity command to robot
            controller.speedL(total_v_desired.tolist())

            # Refresh visualization at a moderate rate to keep control loop responsive
            if t - last_plot_t > 0.03:
                controller.update_3d_trajectory()
                last_plot_t = t
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
    print("Elapsed time:", time.time() - start_t)

if __name__ == "__main__":

    controller = RobotVelocityController()
    controller.start()

    point_list = np.array([[150, 500, 500],
    [50, 500, 500],
    [50, 600, 500],
    [150, 600, 500],
    [150, 600, 400],
    [150, 500, 500],
    [150, 500, 500]]) / 1000.0

    pointing_vectors = np.array([[0, 0, -1],
    [0, 1, 0],
    [0, 1, 0],
    [0, 1, 0], 
    [0, 1, 0],
    [0, 1, 0],
    [0, 0, -1]])

    # Define x-directions for each waypoint
    x_directions = np.array([[1, 0, 0],
    [1, 0, 0],
    [0, 0, -1],
    [1, 0, 0],
    [1, 0, 0],
    [1, 0, 0],
    [1, 0, 0]])

    point_list = np.column_stack((point_list, np.arange(point_list.shape[0])))
        
    duration_list = [10, 20, 30, 40, 50, 60]

    execute_trajectory(controller, point_list, pointing_vectors, x_directions, duration_list)

    # Stop the robot
    controller.stop()

    # Keep the plot window open
    controller.keep_plot_open()

    # Close the connection
    controller.close()