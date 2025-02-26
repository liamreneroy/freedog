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

# Create numpy array for the action space
motion_parameters = {
    "0": ["user", "object"],
    "1": ["left", "neutral", "right"],
    "2": ["backward", "neutral", "forward"],
    "3": ["low", "neutral", "high"],
    "4": ["smooth", "shaky"],
    "5": ["slow", "medium", "fast"]
    }

# create an array for the action space with lists of motion parameters 
action_space = np.empty((2, 3, 3, 3, 2, 3), dtype=object)
for idx in np.ndindex(action_space.shape):
    action_space[idx] = []


for idx in np.ndindex(action_space.shape):
    # Map each index of the tuple to the corresponding motion parameter.
    P1 = motion_parameters["0"][idx[0]]
    P2 = motion_parameters["1"][idx[1]]
    P3 = motion_parameters["2"][idx[2]]
    P4 = motion_parameters["3"][idx[3]]
    P5 = motion_parameters["4"][idx[4]]
    P6 = motion_parameters["5"][idx[5]]

    print(f"Index {idx}: {P1}, {P2}, {P3}, {P4}, {P5}, {P6}")



def main():

    # Initialize connection and highCmd, highState objects - Printer Options: 'all', 'minimal', None
    motor_control_obj = chcf.MotorControl(printer='minimal')
    # Parse data
    motor_control_obj.parse_data()

    for idx in np.ndindex(action_space.shape):
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


        # Add the list "angle_height_log_list" as the entry to action_space at the entry (1, 2, 1, 2, 1, 1)
        action_space[idx] = angle_height_log_list

        # # Test that it worked by printing the list at the entry (1, 2, 1, 2, 1, 1)
        # print("\n::: Testing Action Space Entry\n")
        # print("::: Action Space Entry (1, 2, 1, 2, 1, 1)\n")
        # print(*action_space[idx][20].items(), sep='\n')


    # # Save the action space to to a numpy file
    print("\n::: Saving Action Space to File\n")
    np.save(file='action_space.npy', arr=action_space, allow_pickle=True)
    print("\n::: Action Space Saved\n")



    

    # ToDo 
    # 1. Create a numpy array for the shape of the action space
    # 2. Iterate through the action space combinations, populate numpy array with the lists of log entries
    # 3. Edit the lists with front/back blanks to ensure they are all the same length
    # 4. Save the numpy array to a file

    # # Convert log list to flattened numpy array
    # angle_height_log_np = udf.log_list_to_np(angle_height_log_list)
    # print("\n::: Log List Converted to Numpy Array\n")
    # print(f"::: Numpy Array Shape: {angle_height_log_np.shape}\n")
    # print("::: Numpy Array\n")
    # print(angle_height_log_np)
    # print("\n::: Numpy Array Saved to 'angle_height_log_np.npy'\n")
    # # Save numpy array
    # udf.save_np_array(angle_height_log_np, 'angle_height_log_np.npy')


if __name__ == "__main__":
    main()

