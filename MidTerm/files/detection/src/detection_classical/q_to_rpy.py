import numpy as np

#Code to convert quaternion to euler angles

def quaternion_to_euler(q):

    q_w, q_x, q_y, q_z = q
    
    # Rotation matrix elements
    R11 = 1 - 2 * (q_y**2 + q_z**2)
    R12 = 2 * (q_x * q_y - q_w * q_z)
    R13 = 2 * (q_x * q_z + q_w * q_y)
    
    R21 = 2 * (q_x * q_y + q_w * q_z)
    R22 = 1 - 2 * (q_x**2 + q_z**2)
    R23 = 2 * (q_y * q_z - q_w * q_x)
    
    R31 = 2 * (q_x * q_z - q_w * q_y)
    R32 = 2 * (q_y * q_z + q_w * q_x)
    R33 = 1 - 2 * (q_x**2 + q_y**2)
    
    # Calculate Euler angles
    roll = np.arctan2(R32, R33)  # phi
    pitch = -np.arcsin(R31)      # theta
    yaw = np.arctan2(R21, R11)   # psi
    
    return roll, pitch, yaw

