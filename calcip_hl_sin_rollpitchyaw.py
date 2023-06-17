from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType, SpeedLevel
from ucl.complex import motorCmd, led
import time
import math
import calcip_hl_control_functs as chcf


def main():
    # Initialize connection and highCmd, highState objects
    connect_obj, highCmd_obj, highState_obj = chcf.initialize_control()

    # Parse data
    chcf.parse_data(conn=connect_obj, hstate=highState_obj, print_out=True)

    # Recover Control
    chcf.recover_control(conn=connect_obj, hcmd=highCmd_obj)

    # Control loop
    terminate = False
    while terminate == False:

        # Parse data    
        chcf.parse_data(conn=connect_obj, hstate=highState_obj)

        # Control commands here
        chcf.sin_roll(conn=connect_obj, hcmd=highCmd_obj)
        time.sleep(2)
        chcf.sin_roll(conn=connect_obj, hcmd=highCmd_obj, frequency=2)
        time.sleep(2)  

        chcf.sin_pitch(conn=connect_obj, hcmd=highCmd_obj)
        time.sleep(2)
        chcf.sin_pitch(conn=connect_obj, hcmd=highCmd_obj, frequency=2)
        time.sleep(2)  

        chcf.sin_yaw(conn=connect_obj, hcmd=highCmd_obj)
        time.sleep(2)
        chcf.sin_yaw(conn=connect_obj, hcmd=highCmd_obj, frequency=2)
        time.sleep(2)  

        # Terminate control
        terminate = chcf.terminate_control(conn=connect_obj, hcmd=highCmd_obj)


if __name__ == "__main__":
    main()
