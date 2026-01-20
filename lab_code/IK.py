from time import time
from controller import RobotVelocityController
import numpy as np

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

controller = RobotVelocityController()
controller.start()

# Parameters
acceleration = 0.5
dt = 1.0/500  # 2ms

# [vx, vy, vz, wx, wy, wz]
v_desired = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

for i in range(500):    
    q_current = controller.get_current_q()
    
    q_dot = inverse_velocity_kinematics(q_current, v_desired)
    
    controller.speedJ(q_dot.tolist())
    
    if i % 100 == 0:
        # q_dot = inverse_velocity_kinematics(q_current, v_desired)

        # tcp = forward_kinematics(q_current)
        # print(tcp)
        tcp_pose = controller.get_current_tcp_pose()
        print(f"Iteration {i}, TCP position: [{tcp_pose[0]:.4f}, {tcp_pose[1]:.4f}, {tcp_pose[2]:.4f}]")

print("Control complete, stopping robot")
controller.stop()
controller.close()