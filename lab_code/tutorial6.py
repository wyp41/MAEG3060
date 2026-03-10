import time
from controller import RobotVelocityController
import numpy as np
from tutorial3 import cubic_trajectory, multiple_cubic_coeffs

def rotation_matrix_to_angle_axis(R):
    """Convert rotation matrix to angle-axis representation"""
    cos_angle = (np.trace(R) - 1) / 2
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    
    if angle < 1e-6:
        return np.zeros(3)
    
    # Calculate rotation axis from rotation matrix
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2 * np.sin(angle))
    
    return angle * axis

def angle_axis_to_rotation_matrix(angle_axis):
    """Convert angle-axis representation to rotation matrix"""
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
    """Calculate angle-axis representation from pointing vectors and x-directions"""
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

    # calculate transformation in angle-axis format for each segment
    angle_axis_list = []

    for i in range(len(pointing_vectors)-1):
        delta_angle_axis = pointing_vector_to_angle_axis(
            pointing_vectors[i], pointing_vectors[i+1],
            x_directions[i], x_directions[i+1]
        )      
        angle_axis_list.append(delta_angle_axis)
    
    coeffs_list = multiple_cubic_coeffs(point_list, duration_list, interm_type="zero_velocity")

    controller.init_3d_visualization(point_list, pointing_vectors, x_directions)
    
    last_plot_t = 0.0

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

            cubic_coeffs = coeffs_list[stage]
            target_pos, target_speed = cubic_trajectory(cubic_coeffs, stage_t)

            w_desired = target_speed[3] * angle_axis_list[stage]
            # print("w_desired:", w_desired)

            v_desired = target_speed[:3]

            total_v_desired = np.hstack((v_desired, w_desired))

            tcp_pose_est = controller.tcp_pose_estimate(v_desired, w_desired, t)

            controller.speedL(total_v_desired.tolist())

            # Refresh visualization at a moderate rate to keep control loop responsive.
            if t - last_plot_t > 0.03:
                controller.update_3d_trajectory(tcp_pose=tcp_pose_est)
                last_plot_t = t
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
    print("Elapsed time:", time.time() - start_t)

if __name__ == "__main__":

    # controller = RobotVelocityController(server_ip="127.0.0.1", direct_connect=True)
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
    [1, 0, 0],
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