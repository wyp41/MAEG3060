from time import time
from controller import RobotVelocityController
import numpy as np
from tutorial3 import *

a = [0, 0, 0.425, 0.3922, 0, 0]
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
alpha = [0, -np.pi/2, 0, 0, np.pi/2, -np.pi/2]

def rotation_matrix_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def rotation_matrix_x(alpha):
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha), 0],
        [0, np.sin(alpha), np.cos(alpha), 0],
        [0, 0, 0, 1]
    ])


def forward_kinematics(q):
    print(q)
    q_mod = q.copy()
    q_mod[0] += np.pi
    q_mod[3] += np.pi
    
    T = np.eye(4)
    
    for i in range(6):
        T_i = np.array([np.cos(q_mod[i]), -np.sin(q_mod[i]), 0, a[i],
                        np.sin(q_mod[i])*np.cos(alpha[i]), np.cos(q_mod[i])*np.cos(alpha[i]), -np.sin(alpha[i]), -d[i]*np.sin(alpha[i]),
                        np.sin(q_mod[i])*np.sin(alpha[i]), np.cos(q_mod[i])*np.sin(alpha[i]), np.cos(alpha[i]), d[i]*np.cos(alpha[i]),
                        0, 0, 0, 1]).reshape(4, 4)
        T = T @ T_i
    
    return T

def geometric_jacobian(q):
    q_mod = q.copy()
    q_mod[0] += np.pi
    q_mod[3] += np.pi
    
    J = np.zeros((6, 6))
    T = [np.eye(4)]
    
    for i in range(6):
        T_i = np.array([np.cos(q_mod[i]), -np.sin(q_mod[i]), 0, a[i],
                        np.sin(q_mod[i])*np.cos(alpha[i]), np.cos(q_mod[i])*np.cos(alpha[i]), -np.sin(alpha[i]), -d[i]*np.sin(alpha[i]),
                        np.sin(q_mod[i])*np.sin(alpha[i]), np.cos(q_mod[i])*np.sin(alpha[i]), np.cos(alpha[i]), d[i]*np.cos(alpha[i]),
                        0, 0, 0, 1]).reshape(4, 4)
        T.append(T[-1] @ T_i)

    p_e = T[6][:3, 3]
    
    for i in range(6):
        z_i = T[i+1][:3, 2]
        p_i = T[i+1][:3, 3]
        
        J[:3, i] = np.cross(z_i, p_e - p_i)
        J[3:, i] = z_i
    
    return J


def inverse_velocity_kinematics(q, v_desired):
    J = geometric_jacobian(q)
    J_pinv = np.linalg.pinv(J)
    q_dot = J_pinv @ v_desired
    
    return q_dot

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

if __name__ == "__main__":

    controller = RobotVelocityController(server_ip="127.0.0.1", direct_connect=True)
    controller.start()

    init_tcp = controller.get_current_tcp_pose()
    init_point = init_tcp[:3]
    init_angle_axis = init_tcp[3:]

    point_list = np.array([[150, 500, 500, 0],
    [50, 500, 500, 1000],
    [50, 600, 500, 2000],
    [150, 600, 500, 3000],
    [150, 600, 400, 4000],
    [150, 500, 500, 5000],
    [150, 500, 500, 6000]]) / 1000.0

    # pointing_vectors = np.array([[0, 0, -1],
    # [0, 1, 0],
    # [0, 1, 0],
    # [0, 1, 0], 
    # [0, 1, 0],
    # [0, 0, -1]])

    # # Define x-directions for each waypoint
    # x_directions = np.array([[1, 0, 0],
    # [1, 0, 0],
    # [1, 0, 0],
    # [1, 0, 0],
    # [1, 0, 0],
    # [1, 0, 0]])

    pointing_vectors = np.array([[0, 0, -1],
        [-0.078995,-0.291043,-0.953443],
        [0.081981,-0.072626,-0.993984],
        [0.024405,-0.410699,-0.911445],
        [-0.614444,-0.452743,-0.646129],
        [-0.699867,-0.351211,-0.621962],
        [0, 0, -1]])
    
    x_directions = np.array([[1, 0, 0],
        [0.074018,-0.955504,0.285539],
        [0.979293,0.191109,0.066806],
        [0.826,-0.505294,0.249803],
        [-0.32551,-0.600535,0.730343],
        [-0.470729,-0.428122,0.771444],
        [1, 0, 0]])

    angle_axis_list = []

    for i in range(len(pointing_vectors)-1):
        delta_angle_axis = pointing_vector_to_angle_axis(
            pointing_vectors[i], pointing_vectors[i+1],
            x_directions[i], x_directions[i+1]
        )      
        angle_axis_list.append(delta_angle_axis)

    print("Angle-axis list:", angle_axis_list)

    # point_list = np.hstack((point_list, np.array(angle_axis_list)))
        
    duration_list = [10, 20, 30, 40, 50, 60]

    coeffs_list = multiple_cubic_coeffs(point_list, duration_list, interm_type="zero_velocity")

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

            q_current = controller.get_current_q()
        
            q_dot = inverse_velocity_kinematics(q_current, total_v_desired)
            
            # controller.draw_trajectory(v_desired, np.array([0]*6), t)
            controller.speedJ(q_dot.tolist())
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
    print("Elapsed time:", time.time() - start_t)

    controller.close()