from time import time
from controller import RobotVelocityController
import numpy as np
from tutorial3 import *

# a = [0, 0, 0.6127, 0.57155, 0, 0]
# d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]
# alpha = [0, -np.pi/2, 0, 0, np.pi/2, -np.pi/2]

a = [0, 0, 0.24365, 0.21325, 0, 0]
d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
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

def pointing_vector_to_angle_axis(pointing_vector1, pointing_vector2):
    z_axis1 = pointing_vector1 / np.linalg.norm(pointing_vector1)
    z_axis2 = pointing_vector2 / np.linalg.norm(pointing_vector2)
    
    # Calculate the angle between the two vectors
    cos_angle = np.clip(np.dot(z_axis1, z_axis2), -1.0, 1.0)
    angle = np.arccos(cos_angle)
    
    # Calculate the rotation axis (cross product)
    rotation_axis = np.cross(z_axis1, z_axis2)
    
    if np.linalg.norm(rotation_axis) < 1e-6:
        return np.zeros(3)  # No rotation needed
    
    rotation_axis /= np.linalg.norm(rotation_axis)  # Normalize the rotation axis
    
    # Angle-axis representation is the angle multiplied by the rotation axis
    angle_axis = angle * rotation_axis
    
    return angle_axis

point_list = np.array([[-212, 126, 85],
[-456, 136, -142]]) / 1000.0

duration_list = [10]

controller = RobotVelocityController(server_ip="127.0.0.1")
controller.start()

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

        v_desired = np.array([target_speed[0], target_speed[1], target_speed[2], 0.0, 0.0, 0.0])

        q_current = controller.get_current_q()
    
        q_dot = inverse_velocity_kinematics(q_current, v_desired)
        
        # controller.draw_trajectory(v_desired, np.array([0]*6), t)
        controller.speedJ(q_dot.tolist())
except KeyboardInterrupt:
    pass
finally:
    controller.stop()
print("Elapsed time:", time.time() - start_t)

controller.close()