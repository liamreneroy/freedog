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



def main():
    # Initialize connection and highCmd, highState objects - Printer Options: 'all', 'minimal', None
    motor_control_obj = chcf.MotorControl(printer='minimal')
    # Parse data
    motor_control_obj.parse_data()


    # Get user to hit enter to continue
    print("WARNING: Ensure robot is placed in an open space\nType a BPM then hit 'enter' to continue...")
    bpm_input = input()

    # Check if input can be converted to float to ensure its a number
    try:
        bpm_input = float(bpm_input)
    except ValueError:
        bpm_input = None

    # Uses input BPM factored with BPM limiter, otherwise defaults to 30
    if bpm_input:
        control_bpm = bpm_input
        while control_bpm > motor_control_obj.bpm_limiter:     
            control_bpm /= 2. 
        print(f'\n::: Input [{bpm_input} BPM] factored to [{control_bpm} BPM] with [{motor_control_obj.bpm_limiter} MAX] limiter\n')

    else:
        control_bpm = 60. # THIS IS DIFFERENT THAN OTHER SCRIPS
        print(f'\n::: Default [{control_bpm} BPM] being used\n')


    # Control loop

    angle_height_log_list = motor_control_obj.pose_ctrl(mode='default', publish_hz=200, 
                                    bpm=60, bars=4, loop_repeats=1,
                                    force_bpm_limiter=60,
                                    use_param_time=True,
                                    delay_start=0.0,  
                                    sleep_override=None,
                                    move_to_pose_base_time = 1.5,
                                    pose_raw_param_dict = {'roll': 'neutral', 
                                                        'pitch': 'neutral', 
                                                        'yaw': 'neutral', 
                                                        'body_height': 'neutral', 
                                                        'body_direction': 'user',
                                                        'pose_duration': 'medium',
                                                        'velocity': 'medium',
                                                        'smoothness': 'smooth'},
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

