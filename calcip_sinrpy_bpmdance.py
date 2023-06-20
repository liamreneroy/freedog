from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType, SpeedLevel
from ucl.complex import motorCmd, led

import calcip_hl_control_functs as chcf

import time
import math
import numpy as np


def main():
    # Initialize connection and highCmd, highState objects
    connect_obj, highCmd_obj, highState_obj = chcf.initialize_control()

    # Parse data
    chcf.parse_data(conn=connect_obj, hstate=highState_obj, print_out=True)

    # Recover Control
    chcf.recover_control(conn=connect_obj, hcmd=highCmd_obj)

    # Get user to hit enter to continue
    print("WARNING: Ensure robot is placed in an open space\nHit 'enter' to continue...")
    input()
    
    # Control loop
    terminate = False
    while terminate == False:

        # Parse data    
        chcf.parse_data(conn=connect_obj, hstate=highState_obj)

        # Control commands here (see calcip_hl_control_functs.py)
        chcf.sin_rollpitchyaw(connect_obj, highCmd_obj, publish_hz=200, sleep_override=None, loop_repeats=6, 
                     rollpitchyaw_array=np.array([False, True, True]),
                     amplitude_array=np.array([0.5, 0.5, 0.4]),
                     offset_array=np.array([0, 0, 0]), 
                     period_array=np.array([1, 0.5, 1]), 
                     phase_array=np.array([0, -math.pi/4, 0]), 
                     dev_check=True,
                     printer=True)
        time.sleep(3)  

        # Terminate control
        terminate = chcf.terminate_control(conn=connect_obj, hcmd=highCmd_obj)


if __name__ == "__main__":
    main()
