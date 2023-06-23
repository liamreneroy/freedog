from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType, SpeedLevel
from ucl.complex import motorCmd, led

import calcip_hl_motorctrl_dev as chcf

import time
import math
import numpy as np


def main():
    # Initialize connection and highCmd, highState objects
    motor_control_obj = chcf.MotorControl()

    # Parse data
    motor_control_obj.parse_data(print_out=True)

    # Recover Control
    motor_control_obj.recover_control()

    # Get user to hit enter to continue
    print("WARNING: Ensure robot is placed in an open space\nHit 'enter' to continue...")
    input()
    
    # Control loop
    terminate = False
    while terminate == False:

        # Parse data    
        motor_control_obj.parse_data(print_out=True)

        # Control commands here (see calcip_hl_control_functs.py)
        motor_control_obj.euler_control(mode='default', ctrl_function=math.sin, 
                                publish_hz=200, bpm=45, 
                                sleep_override=None, loop_repeats=8, 
                                euler_array=np.array([1, 1, 1]),
                                amplitude_array=np.array([0.5, 0.5, 0.4]),
                                offset_array=np.array([0, 0, 0]), 
                                period_array=np.array([1, 2, 1]), 
                                phase_array=np.array([2, 2, 2]), 
                                dev_check=True,
                                printer=True)
        time.sleep(2)


        motor_control_obj.euler_control(mode='default', ctrl_function=math.cos, 
                                publish_hz=200, bpm=45, 
                                sleep_override=None, loop_repeats=8, 
                                euler_array=np.array([1, 1, 1]),
                                amplitude_array=np.array([0.5, 0.5, 0.4]),
                                offset_array=np.array([0, 0, 0]), 
                                period_array=np.array([1, 2, 1]), 
                                phase_array=np.array([2, 2, 2]), 
                                dev_check=True,
                                printer=True)
        time.sleep(2)



        motor_control_obj.euler_control(mode='default', ctrl_function=math.cos, 
                                publish_hz=200, bpm=45, 
                                sleep_override=None, loop_repeats=8, 
                                euler_array=np.array([1, 1, 1]),
                                amplitude_array=np.array([0.5, 0.5, 0.4]),
                                offset_array=np.array([0, 0, 0]), 
                                period_array=np.array([1, 2, 1]), 
                                phase_array=np.array([-1/4, -1/2, -1/4]), 
                                dev_check=True,
                                printer=True)
        time.sleep(2)




        motor_control_obj.euler_control(mode='default', ctrl_function=math.cos, 
                                publish_hz=200, bpm=45, 
                                sleep_override=None, loop_repeats=8, 
                                euler_array=np.array([1, 1, 1]),
                                amplitude_array=np.array([0.5, 0.5, 0.4]),
                                offset_array=np.array([0, 0, 0]), 
                                period_array=np.array([1, 2, 1]), 
                                phase_array=np.array([1/2, 1/2, 1/2]), 
                                dev_check=True,
                                printer=True)
        time.sleep(2)




        # motor_control_obj.euler_control(mode='dance', ctrl_function=math.sin, 
        #                         publish_hz=200, bpm=45, 
        #                         sleep_override=None, loop_repeats=16, 
        #                         euler_array=np.array([1, 1, 0]),
        #                         amplitude_array=np.array([0.5, 0.5, 0.4]),
        #                         offset_array=np.array([0, 0, 0]), 
        #                         period_array=np.array([1, 2, 1]), 
        #                         phase_array=np.array([2, 2, 2]), 
        #                         dev_check=True,
        #                         printer=True)
        # time.sleep(2)


        # motor_control_obj.euler_control(mode='dance', ctrl_function=math.sin, 
        #                         publish_hz=200, bpm=65, 
        #                         sleep_override=None, loop_repeats=16, 
        #                         euler_array=np.array([1, 1, 0]),
        #                         amplitude_array=np.array([0.5, 0.5, 0.4]),
        #                         offset_array=np.array([0, 0, 0]), 
        #                         period_array=np.array([1, 2, 1]), 
        #                         phase_array=np.array([0, 0, 0]), 
        #                         dev_check=True,
        #                         printer=True)
        # time.sleep(2)



        # Terminate control
        terminate = motor_control_obj.terminate_control()


if __name__ == "__main__":
    main()
