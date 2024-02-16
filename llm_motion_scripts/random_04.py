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
    motor_control_obj.pose_ctrl(mode='default', publish_hz=200, 
                                    bpm=60, bars=4, loop_repeats=1,
                                    force_bpm_limiter=60,
                                    use_param_time=True,
                                    delay_start=0.0,  
                                    sleep_override=None,
                                    move_to_pose_base_time = 1.5,
                                    pose_raw_param_dict = {'roll': 'left', 
                                                        'pitch': 'neutral', 
                                                        'yaw': 'neutral', 
                                                        'body_height': 'low', 
                                                        'body_direction': 'user',
                                                        'pose_duration': 'medium',
                                                        'velocity': 'fast',
                                                        'smoothness': 'smooth'},
                                    dev_check=None)



if __name__ == "__main__":
    main()

