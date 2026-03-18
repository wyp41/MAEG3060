from time import time
import numpy as np

def rotation_matrix_x(alpha):
    '''
    Returns the rotation matrix for a rotation about the x-axis.
    Input:
        alpha: the angle of rotation about the x-axis in radians
    Output:        
        A 4x4 homogeneous transformation matrix representing the rotation
    '''
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha), 0],
        [0, np.sin(alpha), np.cos(alpha), 0],
        [0, 0, 0, 1]
    ])

def translation_matrix_x(x):
    '''
    Returns the translation matrix for a translation along the x-axis.
    Input:
        x: the distance of translation along the x-axis
    Output:
        A 4x4 homogeneous transformation matrix representing the translation
    '''
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def rotation_matrix_z(theta):
    '''
    Returns the rotation matrix for a rotation about the z-axis.
    Input:
        theta: the angle of rotation about the z-axis in radians
    Output:
        A 4x4 homogeneous transformation matrix representing the rotation
    '''
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def translation_matrix_z(z):
    '''
    Returns the translation matrix for a translation along the z-axis.
    Input:
        z: the distance of translation along the z-axis
    Output:
        A 4x4 homogeneous transformation matrix representing the translation
    '''
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def forward_kinematics(q, alpha, a, theta, d):
    q_modified = q.copy()
    q_modified[1] += np.pi/2
    q_modified[3] += np.pi/2
    q_modified += np.array(theta)
    
    T = np.eye(4)
    
    for i in range(6):
        T_i = rotation_matrix_x(alpha[i]) @ translation_matrix_x(a[i]) @ rotation_matrix_z(q_modified[i]) @ translation_matrix_z(d[i])
        T = T @ T_i
    
    return T

if __name__ == "__main__":

    # Define the DH parameters for the robot, these should be replaced with the actual parameters of your robot
    a = [0, 0, 0, 0, 0, 0]
    d = [0, 0, 0, 0, 0, 0]
    alpha = [0, 0, 0, 0, 0, 0]
    theta = [0, 0, 0, 0, 0, 0]

    q1 = np.array([100, 20, 20, 30, 40, 50])
    q1 = np.deg2rad(q1)
    T1 = forward_kinematics(q1, alpha, a, theta, d)

    q2 = np.array([120, 40, 40, 50, 60, 70])
    q2 = np.deg2rad(q2)
    T2 = forward_kinematics(q2, alpha, a, theta, d)

    q3 = np.array([140, 60, 60, 70, 80, 90])
    q3 = np.deg2rad(q3)
    T3 = forward_kinematics(q3, alpha, a, theta, d)

    q4 = np.array([0,0,0,0,0,0])
    q4 = np.deg2rad(q4)
    T4 = forward_kinematics(q4, alpha, a, theta, d)

    q5 = np.array([0, -90, 0, -90, 0, 0])
    q5 = np.deg2rad(q5)
    T5 = forward_kinematics(q5, alpha, a, theta, d)

    # Check the results against the correct transformation matrices

    correct_T = {1:
                [[ 0.5026544,  -0.34518353,  0.79258242,  0.22975339],
                [-0.4713095,  -0.87800784, -0.08348413, -0.29469861],
                [ 0.72471092, -0.33158796, -0.60402277, -0.14716855],
                [ 0.         ,  0.         ,  0.         ,  1.        ]],
                2:
                [[ 0.67139984, -0.72477384,  0.1546775,   0.1891137 ],
                [-0.57050237, -0.37225024,  0.73209071, -0.02095453],
                [-0.47302146, -0.57976947, -0.66341395, -0.21419713],
                [ 0.          ,  0.          ,  0.          ,  1.        ]],
                3:
                [[-0.13302222, -0.76402354, -0.63132648,  0.04350887],
                [ 0.1116189,  -0.64448335,  0.75642741,  0.12871945],
                [-0.98480775,  0.03015369,  0.17101007, -0.14572794],
                [ 0.          ,  0.          ,  0.          ,  1.        ]],
                4:
                [[ 1.0000000e+00,  1.2246468e-16, -1.2246468e-16, -4.5690000e-01],
                [-1.2246468e-16,  6.1232340e-17, -1.0000000e+00, -1.9425000e-01],
                [-1.2246468e-16,  1.0000000e+00,  6.1232340e-17,  6.6550000e-02],
                [ 0.0000000e+00,  0.0000000e+00,  0.0000000e+00,  1.0000000e+00]],
                5:
                [[-1.00000000e+00, -1.23259516e-32, -1.22464680e-16, -5.17658202e-17],
                [ 1.22464680e-16, -6.12323400e-17, -1.00000000e+00, -1.94250000e-01],
                [ 0.00000000e+00, -1.00000000e+00,  6.12323400e-17,  6.94150000e-01],
                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]}

    for i in range(1, 6):
        T = eval(f"T{i}")
        print(f"T{i} - correct_T[{i}]:\n{np.round(T - correct_T[i], 2)}")