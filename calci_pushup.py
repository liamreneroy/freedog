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
import math
import numpy as np


# README:
# Run this script at 108.85 bpm to 'Eye of the Tiger'


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
        control_bpm = 30.
        print(f'\n::: Default [{control_bpm} BPM] being used\n')



    # Control loop
    time.sleep(2./control_bpm * 60) # sleep for 2 beats at BPM rate

    # Do some push ups
    motor_control_obj.push_up_ctrl(mode='default', publish_hz=200, bpm=control_bpm, 
                                    sleep_override=None, loop_repeats=6,  
                                    dev_check=None)
    
    # Look over to the side
    motor_control_obj.sin_euler_ctrl(mode='default', publish_hz=200, bpm=control_bpm, 
                                    sleep_override=None, loop_repeats=2, 
                                    euler_array=np.array([1, 0, 1]),
                                    sin_func_array=np.array([math.sin, math.sin, math.sin]),
                                    amplitude_array=np.array([0.5, 0.5, 0.4]),
                                    offset_array=np.array([0.0, 0.0, 0.0]), 
                                    period_array=np.array([4.0, 4.0, 4.0]), 
                                    phase_array=np.array([0.0, 0.0, 0.0]), 
                                    dev_check=None)

    # Do some more push ups
    motor_control_obj.push_up_ctrl(mode='default', publish_hz=200, bpm=control_bpm, 
                                    sleep_override=None, loop_repeats=6,  
                                    dev_check=None)
    
    # Look up
    motor_control_obj.sin_euler_ctrl(mode='default', publish_hz=200, bpm=control_bpm, 
                                    sleep_override=None, loop_repeats=2, 
                                    euler_array=np.array([0, 1, 0]),
                                    sin_func_array=np.array([math.sin, math.sin, math.sin]),
                                    amplitude_array=np.array([0.5, -0.5, 0.4]),
                                    offset_array=np.array([0.0, 0.0, 0.0]), 
                                    period_array=np.array([1.0, 4.0, 1.0]), 
                                    phase_array=np.array([0.0, 0.0, 0.0]), 
                                    dev_check=None)

    # Dance it out
    motor_control_obj.sin_euler_ctrl(mode='default', publish_hz=200, bpm=control_bpm, 
                                    sleep_override=None, loop_repeats=8, 
                                    euler_array=np.array([1, 1, 1]),
                                    sin_func_array=np.array([math.sin, udf.neg_abs_sin, udf.neg_sin]),
                                    amplitude_array=np.array([0.3, 0.45, 0.4]),
                                    offset_array=np.array([0.0, 0.0, 0.0]), 
                                    period_array=np.array([2.0, 2.0, 1.0]), 
                                    phase_array=np.array([0.0, 0.0, 0.0]), 
                                    dev_check=None)


if __name__ == "__main__":
    main()

