from time import time
from controller import RobotVelocityController
import numpy as np
from tutorial3 import *

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

def angular_velocity_from_pointing_vector(pointing_vector1, pointing_vector2, dt):
    angle_axis = pointing_vector_to_angle_axis(pointing_vector1, pointing_vector2)
    angle = np.linalg.norm(angle_axis)
    
    if angle < 1e-6:
        return np.zeros(3)  # No rotation needed
    
    rotation_axis = angle_axis / angle  # Normalize the rotation axis
    
    # Angular velocity is the angle divided by time, along the rotation axis
    angular_velocity = (angle / dt) * rotation_axis
    
    return angular_velocity
    

controller = RobotVelocityController("127.0.0.1", direct_connect=True)
controller.start()

# Get initial TCP position (this will be the top of the circle where y is maximum)
tcp_pose = controller.get_current_tcp_pose()
x0, y0, z0 = tcp_pose[0], tcp_pose[1], tcp_pose[2]

# Get current angle-axis from TCP pose (rx, ry, rz)
angle_axis = np.array([tcp_pose[3], tcp_pose[4], tcp_pose[5]])
pointing_vector = angle_axis  # Initial pointing vector
print(f"Starting position: x={x0:.4f}, y={y0:.4f}, z={z0:.4f}")
print(f"Starting angle-axis: rx={angle_axis[0]:.4f}, ry={angle_axis[1]:.4f}, rz={angle_axis[2]:.4f}")

target_position = np.array([x0, y0, z0-0.2])  # Move down by 20cm
target_pointing_vector = np.array([0, 0, -1])  # Pointing straight down
angle_axis_target = pointing_vector_to_angle_axis(pointing_vector, target_pointing_vector)
target_pose = np.concatenate((target_position, angle_axis_target))

duration_list = [10.0]  # seconds

point_list = [np.array([x0, y0, z0, 0, 0, 0], target_pose)]
coeffs_list = multiple_cubic_coeffs(point_list, duration_list, interm_type="zero_velocity")

start_time = time()

while True:
    t = time() - start_time
    
    # Stop after one complete circle
    if t >= duration_list[-1]:
        break
    cubic_coeffs = coeffs_list[0]
    target_pos, target_speed = cubic_trajectory(cubic_coeffs, t)
    v_desired = target_speed
    
    # Get current TCP position
    tcp_pose = controller.get_current_tcp_pose()
    current_x, current_y, current_z = tcp_pose[0], tcp_pose[1], tcp_pose[2]
    
    # Get current joint angles
    q_current = controller.get_current_q()
    
    # Calculate joint velocities using inverse velocity kinematics
    q_dot = inverse_velocity_kinematics(q_current, v_desired)
    
    # Send velocity command
    controller.speedJ(q_dot.tolist())

print("Circle tracking complete, stopping robot")
controller.stop()
controller.close()
