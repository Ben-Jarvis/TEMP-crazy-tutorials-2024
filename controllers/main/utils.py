import numpy as np

# Create a rotation matrix from the world frame to the body frame using euler angles
def euler2rotmat(euler_angles):
    
    R = np.eye(3)
    
    # Here you need to implement the rotation matrix
    # First calculate the rotation matrix for each angle (roll, pitch, yaw)
    # Then multiply the matrices together to get the total rotation matrix

    # --- YOUR CODE HERE ---
    
    
    return R

# Rotate the control commands from the global refernce frame to the drone reference frame
def rot_global2body(control_commands, euler_angles):
    
    # Here you need to rotate the control commands from the global reference frame to the drone reference frame
    # You should use the euler2rotmat function to get the rotation matrix
    # Keep in mind that you only want to rotate the velocity commands, which are the first two elements of the control_commands array

    # --- YOUR CODE HERE ---

    return control_commands