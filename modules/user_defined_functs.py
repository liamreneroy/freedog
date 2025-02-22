import math
import numpy as np

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


# Oscillating control functions
def neg_sin(x):
    return -math.sin(x)

def abs_sin(x):
    return abs(math.sin(x))

def neg_abs_sin(x):
    return -abs(math.sin(x))

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Array functions
def reverse_array(original_array):
    reversed_array = original_array[::-1]
    return reversed_array

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Kinematics functions

# Calculate XYZ locations using forward kinematics and 12 joint angles in np.array
# def forward_kinematics(joint_angles):
#     # Joint angles are in radians

#     # Define robot's dimensions using URDF file
#     # 

#     pass

# # Calculate robot's XYZ joint locations from joint angles
# def get_joint_xyz(self):
#     ''' Get joint XYZ locations from the robot'''

#     joint_angles = self.get_joint_angles()        # Get joint angles

#     # Calculate XYZ locations using forward kinematics and 12 joint angles in np.array
#     joint_xyz = forward_kinematics(joint_angles)

#     return joint_xyz