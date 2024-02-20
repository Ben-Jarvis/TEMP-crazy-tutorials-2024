import numpy as np

# Create a rotation matrix from the world frame to the body frame using euler angles
def euler2rotmat(euler_angles):
    
    R = np.eye(3)
    
    # Here you need to implement the rotation matrix
    # First calculate the rotation matrix for each angle (roll, pitch, yaw)
    # Then multiply the matrices together to get the total rotation matrix

    # --- YOUR CODE HERE ---

    #R_roll = 
    #R_pitch =
    #R_yaw =

    #R = R_yaw @ R_pitch @ R_roll


    # --- SAMPLE SOLUTION ---
    # R_roll = np.array([[1, 0, 0], [0, np.cos(euler_angles[0]), -np.sin(euler_angles[0])], [0, np.sin(euler_angles[0]), np.cos(euler_angles[0])]])
    # R_pitch = np.array([[np.cos(euler_angles[1]), 0, np.sin(euler_angles[1])], [0, 1, 0], [-np.sin(euler_angles[1]), 0, np.cos(euler_angles[1])]])
    # R_yaw = np.array([[np.cos(euler_angles[2]), -np.sin(euler_angles[2]), 0], [np.sin(euler_angles[2]), np.cos(euler_angles[2]), 0], [0, 0, 1]])
    
    # R = R_yaw @ R_pitch @ R_roll
    
    return R

# Rotate the control commands from the global refernce frame to the drone reference frame
def rot_global2body(control_commands, euler_angles):
    
    # Here you need to rotate the control commands from the global reference frame to the drone reference frame
    # You should use the euler2rotmat function to get the rotation matrix
    # Keep in mind that you only want to rotate the velocity commands, which are the first two elements of the control_commands array
    # Think carefully about which direction you need to perform the rotation

    # --- YOUR CODE HERE ---

    # vel_world = 
    
    # R = euler2rotmat(euler_angles)
    
    # vel_body =
    
    # control_commands =
     
    
    # --- SAMPLE SOLUTION ---
    # vel_world = [control_commands[0], control_commands[1], 0] 
    
    # R = euler2rotmat(euler_angles)

    # vel_body = R @ vel_world

    # control_commands[0] = vel_body[0]
    # control_commands[1] = vel_body[1]


    return control_commands

   
