from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType, SpeedLevel
from ucl.complex import motorCmd, led

import modules.calcip_hl_motorctrl_dev as chcf
import modules.user_defined_functs as udf

import time
import math
import numpy as np


# README:
# Run this script to some music and set the BPM accordingly 


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

    # Random dance
    motor_control_obj.sin_euler_ctrl(mode='rand_dance', publish_hz=200, bpm=control_bpm, 
                                    sleep_override=None, loop_repeats=32, 
                                    euler_array=np.array([1, 1, 1]),
                                    sin_func_array=np.array([math.sin, math.sin, math.sin]),
                                    amplitude_array=np.array([0.45, 0.5, 0.4]),
                                    offset_array=np.array([0.0, 0.0, 0.0]), 
                                    period_array=np.array([2.0, 2.0, 2.0]), 
                                    phase_array=np.array([0.0, 0.0, 0.0]), 
                                    dev_check=None)


if __name__ == "__main__":
    main()

