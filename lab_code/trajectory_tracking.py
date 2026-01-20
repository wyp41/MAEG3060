from time import time
from controller import RobotVelocityController
import numpy as np

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

controller = RobotVelocityController("127.0.0.1")
controller.start()

# Get initial TCP position (this will be the top of the circle where y is maximum)
tcp_pose = controller.get_current_tcp_pose()
x0, y0, z0 = tcp_pose[0], tcp_pose[1], tcp_pose[2]
print(f"Starting position: x={x0:.4f}, y={y0:.4f}, z={z0:.4f}")

# Circle parameters
radius = 0.05  # 5cm radius
angular_velocity = 0.3  # rad/s (adjust for desired speed)
kp = 2.0  # Proportional gain for error correction

# Total duration for one complete circle
duration = 2 * np.pi / angular_velocity

print(f"Tracking circular trajectory with radius {radius}m")
print(f"Angular velocity: {angular_velocity} rad/s")
print(f"Duration: {duration:.2f} seconds")

# Circle center is at (x0, y0 - radius, z0)
# This makes current position the top of the circle (y maximum)
cx = x0
cy = y0 - radius
cz = z0

# Start time
start_time = time()
iteration = 0

while True:
    t = time() - start_time
    
    # Stop after one complete circle
    if t >= duration:
        break
    
    theta = angular_velocity * t
    iteration += 1
    
    # Expected position on the circle
    expected_x = cx + radius * np.sin(theta)
    expected_y = cy + radius * np.cos(theta)
    expected_z = cz

    # Feedforward velocity (circular motion)
    v_theory = np.array([radius * angular_velocity * np.cos(theta), -radius * angular_velocity * np.sin(theta), 0.0])
    
    # Get current TCP position
    tcp_pose = controller.get_current_tcp_pose()
    current_x, current_y, current_z = tcp_pose[0], tcp_pose[1], tcp_pose[2]
    
    # Position error
    error_x = expected_x - current_x
    error_y = expected_y - current_y
    error = [error_x, error_y, 0.0]
    
    v_modified = v_theory + kp * np.array(error)
    
    v_desired = np.array([v_modified[0], v_modified[1], v_modified[2], 0.0, 0.0, 0.0])
    
    # Get current joint angles
    q_current = controller.get_current_q()
    
    # Calculate joint velocities using inverse velocity kinematics
    q_dot = inverse_velocity_kinematics(q_current, v_desired)
    
    # Send velocity command
    controller.speedJ(q_dot.tolist())
    
    # Print progress every 100 iterations
    if iteration % 100 == 0:
        tcp_pose = controller.get_current_tcp_pose()
        expected_x = cx + radius * np.sin(theta)
        expected_y = cy + radius * np.cos(theta)
        error = np.sqrt((tcp_pose[0] - expected_x)**2 + (tcp_pose[1] - expected_y)**2)
        print(f"t={t:.2f}s, Iteration {iteration}, TCP: [{tcp_pose[0]:.4f}, {tcp_pose[1]:.4f}, {tcp_pose[2]:.4f}], "
              f"Expected: [{expected_x:.4f}, {expected_y:.4f}, {cz:.4f}], Error: {error:.4f}m")

print("Circle tracking complete, stopping robot")
controller.stop()
controller.close()
