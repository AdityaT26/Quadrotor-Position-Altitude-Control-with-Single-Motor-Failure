import numpy as np

def transform_rpy(roll, pitch, yaw, rotation_angle=np.pi/4):
    """
    Transform roll-pitch-yaw angles from one reference frame to another
    that's rotated by rotation_angle radians anticlockwise about Z axis.
    
    Args:
        roll (float): Roll angle in radians
        pitch (float): Pitch angle in radians
        yaw (float): Yaw angle in radians
        rotation_angle (float): Angle between the two reference frames in radians
                              (defaults to π/4 radians = 45 degrees)
    
    Returns:
        tuple: (new_roll, new_pitch, new_yaw) angles in radians
    """
    # Create rotation matrix from original RPY angles
    def rot_x(angle):
        return np.array([
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]
        ])
    
    def rot_y(angle):
        return np.array([
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)]
        ])
    
    def rot_z(angle):
        return np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])
    
    # Original rotation matrix (RPY rotation)
    R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    
    # Transform matrix for rotation about Z-axis
    T = rot_z(rotation_angle)
    
    # New rotation matrix in rotated frame
    R_new = T.T @ R @ T
    
    # Extract new Euler angles
    def rotation_matrix_to_euler_angles(R):
        sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2,1], R[2,2])
            pitch = np.arctan2(-R[2,0], sy)
            yaw = np.arctan2(R[1,0], R[0,0])
        else:
            roll = np.arctan2(-R[1,2], R[1,1])
            pitch = np.arctan2(-R[2,0], sy)
            yaw = 0

        return np.array([roll, pitch, yaw])

    return tuple(rotation_matrix_to_euler_angles(R_new))
