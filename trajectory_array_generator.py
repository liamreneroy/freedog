from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType, SpeedLevel
from ucl.complex import motorCmd, led

import modules.calci_hl_ctrl_dev as chcf
import modules.user_defined_functs as udf

import time
import numpy as np

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# NOTES
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# - This file needs to be run in them same directory as the "data" folder to save and load the .npy files correctly.


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# SETTINGS
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

go1_raw_trajectory_array_filename = 'data/go1_raw_trajectory_array_box_charger.npy'
go1_transformed_trajectory_array_filename = 'data/go1_transformed_trajectory_array_box_charger.npy'


# Create numpy array for the action space
go1_motion_parameters = {
    "0": ["user", "object_left", "object_right"],   # Body Direction
    "1": ["left", "neutral", "right"],              # Body Tilt
    "2": ["backward", "neutral", "forward"],        # Body Lean
    "3": ["low", "neutral", "high"],                # Body Height
    "4": ["smooth", "shaky"],                       # Motion Smoothness
    "5": ["slow", "fast"]                           # Motion Velocity
    }

# Create an array for the action space with lists of motion parameters 
go1_action_space_shape = (3, 3, 3, 3, 2, 2)   # <- shape not indexes


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# FUNCTIONS
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def load_or_generate_action_space(array_filename, motion_parameters, action_space_shape):

    """
    # check if a .npy file with specific name exists in the current directory, if yes, load it and return
    # if not, generate the action space and save it to a .npy file
    # return the action space
    """

    try:
        trajectory_lists = np.load(array_filename, allow_pickle=True)
        print("\n::: Action Space Loaded from File")
        print(f"::: Action Space Shape: {trajectory_lists.shape}\n")
        return trajectory_lists
    
    except:
        print("\n::: No Action Space File Found\n")
        print("\n::: Generating Action Space\n")

        # Initialize connection and highCmd, highState objects - Printer Options: 'all', 'minimal', None
        motor_control_obj = chcf.MotorControl(printer='minimal')
        # Parse data
        motor_control_obj.parse_data()

        trajectory_lists = np.empty(action_space_shape, dtype=object)
        # # Add a list to each entry in the action space
        # for idx in np.ndindex(go1_trajectory_lists.shape):
        #     go1_trajectory_lists[idx] = []

        for idx in np.ndindex(trajectory_lists.shape):
            # Map each index of the tuple to the corresponding motion parameter.
            P1 = motion_parameters["0"][idx[0]]  # Body Direction       aka body_direction
            P2 = motion_parameters["1"][idx[1]]  # Body Tilt            aka roll
            P3 = motion_parameters["2"][idx[2]]  # Body Lean            aka pitch
            P4 = motion_parameters["3"][idx[3]]  # Body Height          aka body_height
            P5 = motion_parameters["4"][idx[4]]  # Motion Smoothness    aka smoothness
            P6 = motion_parameters["5"][idx[5]]  # Motion Velocity      aka velocity

            print(f"**********************************************")
            print(f"Index {idx}: {P1}, {P2}, {P3}, {P4}, {P5}, {P6}")
            print(f"**********************************************")


            # Control loop
            angle_height_log_list = motor_control_obj.pose_ctrl(mode='default', publish_hz=200, 
                                            bpm=60, bars=4, loop_repeats=1,
                                            force_bpm_limiter=60,
                                            use_param_time=True,
                                            delay_start=0.0,  
                                            sleep_override=None,
                                            move_to_pose_base_time = 1.5,
                                            pose_raw_param_dict = {'roll': P2, 
                                                                'pitch': P3, 
                                                                'yaw': 'neutral', 
                                                                'body_height': P4, 
                                                                'body_direction': P1,
                                                                'pose_duration': 'medium',
                                                                'velocity': P6,
                                                                'smoothness': P5},
                                            dev_check=None,
                                            angle_height_logger=True)

            print("\n::: Logging Complete\n")
            print(f"::: Total Log Entries: {len(angle_height_log_list)}\n")
            print("::: First Entry:")
            print(*angle_height_log_list[0].items(), sep='\n')
            print("\n::: 20th Entry\n")
            print(*angle_height_log_list[19].items(), sep='\n')
            print("\n::: Last Entry\n")
            print(*angle_height_log_list[-1].items(), sep='\n')


            # Add the list "angle_height_log_list" as the entry to go1_trajectory_lists at the entry (1, 2, 1, 2, 1, 1)
            trajectory_lists[idx] = angle_height_log_list

        # # Save the action space to to a numpy file
        print(f"\n::: Saving Action Space to File: {array_filename}\n")
        np.save(file=array_filename, arr=trajectory_lists, allow_pickle=True)
        print("\n::: Action Space Saved\n")

        return trajectory_lists


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def refactored_action_space(trajectory_lists_array, hardcode_max_length=None):

    """
    # Take the action space and make all the lists the same length by copying the last element of the list until the length is the same as the max_length
    """

    if hardcode_max_length is not None:
        max_length = hardcode_max_length
    else:
        # Find max length of the trajectory lists
        max_length = 0
        for idx in np.ndindex(trajectory_lists_array.shape):
            if len(trajectory_lists_array[idx]) > max_length:
                max_length = len(trajectory_lists_array[idx])

    # Similarly, find min length of the trajectory lists
    min_length = max_length
    for idx in np.ndindex(trajectory_lists_array.shape):
        if len(trajectory_lists_array[idx]) < min_length:
            min_length = len(trajectory_lists_array[idx])
    
    # Now take every element in the action space and make them all the same length. 
    # Do this by copying the last element of the list until the length is the same as the max_length
    for idx in np.ndindex(trajectory_lists_array.shape):
        while len(trajectory_lists_array[idx]) < max_length:
            trajectory_lists_array[idx].append(trajectory_lists_array[idx][-1])

    print(f"\n::: Min Length of Trajectory Lists: {min_length}")
    print(f"::: Max Length of Trajectory Lists: {max_length}")
    print(f"::: Refactoring all lists to have length: {max_length}")

    # Now check that all the lists are the same length
    for idx in np.ndindex(trajectory_lists_array.shape):
        if len(trajectory_lists_array[idx]) != max_length:
            print(f"\n::: ERROR: List at index {idx} inconsistent from max length: {max_length}\n")
        else:
            pass

    return trajectory_lists_array


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def euler_to_euclidians(trajectory_step):
    
    """
    # Take a trajectory step and convert the euler angles to euclidian coordinates
    """

    euclidian_trajectory_step = []

    for step in trajectory_step:
        euclidian_step = udf.euler_to_euclidian(step)
        euclidian_trajectory_step.append(euclidian_step)

    return euclidian_trajectory_step

import numpy as np

import numpy as np



# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def transform_points(points_dict, transformation_params):
    """
    Transforms a dictionary of 3D points based on roll, pitch, yaw, body height (Z-translation),
    and an optional pre-yaw rotation.

    Parameters:
        points_dict: Dictionary of 3D points {name: (x, y, z)}
        transformation_params: Dictionary containing:
            - 'body_direction': Pre-yaw angle (radians, rotates about Z-axis before other transformations)
            - 'roll': Roll angle (radians, rotation about X-axis)
            - 'pitch': Pitch angle (radians, rotation about Y-axis)
            - 'yaw': Yaw angle (radians, rotation about Z-axis)
            - 'body_height': Z translation (meters, up/down shift)

    Returns:
        Dictionary of transformed 3D points {name: (x', y', z')}
    """
    # Extract transformation parameters
    body_direction = transformation_params.get('body_direction', 0.0)
    roll = transformation_params.get('roll', 0.0)
    pitch = transformation_params.get('pitch', 0.0)
    yaw = transformation_params.get('yaw', 0.0)
    body_height = transformation_params.get('body_height', 0.0)

    # Define Rotation Matrices

    # Roll (X-axis)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Pitch (Y-axis)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Yaw (Z-axis)
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Pre-Yaw Rotation (Rotates around Z-axis by any angle `body_direction`)
    R_preyaw = np.array([
        [np.cos(body_direction), -np.sin(body_direction), 0],
        [np.sin(body_direction), np.cos(body_direction), 0],
        [0, 0, 1]
    ])

    # Process each point in the dictionary
    transformed_points = {}
    for name, (x, y, z) in points_dict.items():
        P = np.array([x, y, z])  # Convert point to numpy array

        # Step 1: Apply Pre-Yaw Rotation
        P = R_preyaw @ P

        # Step 2: Apply Roll (Rotation about X-axis)
        P = R_x @ P

        # Step 3: Apply Pitch (Rotation about Y-axis)
        P = R_y @ P

        # Step 4: Apply Yaw (Rotation about Z-axis)
        P = R_z @ P

        # Step 5: Apply Z Translation (body_height)
        P[2] += body_height

        # Store transformed point in dictionary with new names (QSR, QSL, QHL, QHR)
        transformed_points[name.replace("P", "Q")] = tuple(P)

    return transformed_points

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def load_or_transform_trajectory_array(trajectory_array, neutral_stance, transformed_array_filename):
    """
    Load a transformed action space from a file if it exists, otherwise generate it from the raw action space.
    """


    try:
        transformed_trajectory_array = np.load(transformed_array_filename, allow_pickle=True)
        print("\n::: Transformed Array Loaded from File")
        print(f"::: Transformed Array Shape: {transformed_trajectory_array.shape}\n")
        return transformed_trajectory_array
    
    except:
        print("\n::: No Transformed Array File Found\n")
        print("\n::: Generating Transformed Array\n")



        transformed_trajectory_array = np.empty_like(trajectory_array, dtype=object)


        # For each element in the refactored action space, transform each timestep of each list
        for idx in np.ndindex(trajectory_array.shape):
            print(f"\n::: Transforming Action Space Entry: {idx}\n")

            # Get the list of trajectory steps
            pose_trajectory_list = trajectory_array[idx]

            # Transform each timestep in the list
            euclidean_trajectory_list = [transform_points(neutral_stance, euler_dict_timestep) 
                                        for euler_dict_timestep in pose_trajectory_list]
            
            # Store the transformed trajectory list in the new array
            transformed_trajectory_array[idx] = euclidean_trajectory_list

        # Print pre-and-post transformed points item by item
        print("\n::: Pre-Transformed Points:")
        for point, coordinates in neutral_stance.items():
            print(f"::: {point}: {coordinates}")

        # Print what the entry was at refactored_go1_trajectory_lists for [(1, 2, 1, 2, 1, 1)][20]
        print("\n::: Pre-Transformed Action Space Entry (1, 2, 1, 2, 1, 1)\n")
        print(*trajectory_array[(1, 2, 1, 2, 1, 1)][20].items(), sep='\n')
        
        # Print one element of the transformed action space so we can test that it worked
        print("\n::: Transformed Action Space Entry (1, 2, 1, 2, 1, 1)\n")
        print(*transformed_trajectory_array[(1, 2, 1, 2, 1, 1)][20].items(), sep='\n')

        # Save the transformed action space to a file
        np.save(file=transformed_array_filename, arr=transformed_trajectory_array, allow_pickle=True)
        print(f"\n::: Transformed Action Space Saved to File: {transformed_array_filename}\n")

        return transformed_trajectory_array


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def main():

    raw_go1_trajectory_array = load_or_generate_action_space(array_filename=go1_raw_trajectory_array_filename, 
                                                             motion_parameters=go1_motion_parameters, 
                                                             action_space_shape=go1_action_space_shape)

    # Print one element of the action space so we can test that it worked
    test_idx, test_timestep = (1, 2, 1, 2, 1, 1), 20
    print("::: Testing Action Space Entry")
    print(f"::: Entry {test_idx} @ timestep: {test_timestep * 10}\n")
    print(*raw_go1_trajectory_array[test_idx][test_timestep].items(), sep='\n')

    # Refactor the action space to have all lists the same length
    refactored_go1_trajectory_array = refactored_action_space(raw_go1_trajectory_array)

    go1_neutral_stance = {
        "PFR": (0.1881, -0.04675, 0),  # Front Right Hip (PFR)
        "PFL": (0.1881, 0.04675, 0),   # Front Left Hip (PFL)
        "PRL": (-0.1881, 0.04675, 0),  # Rear Left Hip (PRL)
        "PRR": (-0.1881, -0.04675, 0)  # Rear Right Hip (PRR)
    }

    transformed_go1_trajectory_array = load_or_transform_trajectory_array(trajectory_array=refactored_go1_trajectory_array, 
                                                                          neutral_stance=go1_neutral_stance, 
                                                                          transformed_array_filename=go1_transformed_trajectory_array_filename)

    # Print one element of the transformed action space so we can test that it worked
    print("\n::: Testing Transformed Array Entry\n")
    print(f"::: Entry {test_idx} @ timestep: {test_timestep * 10}\n")
    print(*transformed_go1_trajectory_array[test_idx][test_timestep].items(), sep='\n')

    print("\n::: Script Complete :)\n")

if __name__ == "__main__":
    main()











# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ARCHIVE CODE
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# example_transformation_values = {
# "body_direction": 1.5707963267948966,  # Pre-Yaw (radians) aka rotate body by 90 degrees to the left
# "roll": 0,  # Roll (radians)
# "pitch": 0,  # Pitch (radians)
# "yaw": 0,  # Yaw (radians)
# "body_height": 0.05  # Z-translation (meters)
# }
