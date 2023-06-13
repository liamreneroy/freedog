from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType, SpeedLevel
from ucl.complex import motorCmd
import time


# You can use one of the 3 Presets WIFI_DEFAULTS, LOW_CMD_DEFAULTS or HIGH_CMD_DEFAULTS.
# IF NONE OF THEM ARE WORKING YOU CAN DEFINE A CUSTOM ONE LIKE THIS:
#
# MY_CONNECTION_SETTINGS = (listenPort, addr_wifi, sendPort_high, local_ip_wifi)
# conn = unitreeConnection(MY_CONNECTION_SETTINGS)



print(f'Running lib version: {lib_version()}')

conn = unitreeConnection(HIGH_WIFI_DEFAULTS)    # Creates a new connection object with HIGH_WIFI_DEFAULTS named 'conn'
conn.startRecv()                                # Starts up connection 
hcmd = highCmd()                                # Creates the highCmd object named hcmd
hstate = highState()                            # Creates the highState object named hstate

cmd_bytes = hcmd.buildCmd(debug=False)          # Builds an empty command
conn.send(cmd_bytes)                            # Send empty command to tell the dog the receive port and initialize the connection
time.sleep(0.5)                                 # Some time to collect packets ;)


i = 0
while True:
    data = conn.getData()

    # Print out packet data to the console. Do you want to print any other data?
    for packet in data:
        print(i)
        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
        hstate.parseData(packet)
        print(f'SN [{byte_print(hstate.SN)}]:\t{decode_sn(hstate.SN)}')
        print(f'Ver [{byte_print(hstate.version)}]:\t{decode_version(hstate.version)}')
        print(f'SOC:\t\t\t{hstate.bms.SOC} %')
        print(f'Overall Voltage:\t{getVoltage(hstate.bms.cell_vol)} mv') # something is still wrong here ?!
        print(f'Current:\t\t{hstate.bms.current} mA')
        print(f'Cycles:\t\t\t{hstate.bms.cycle}')
        print(f'Temps BQ:\t\t{hstate.bms.BQ_NTC[0]} °C, {hstate.bms.BQ_NTC[1]}°C')
        print(f'Temps MCU:\t\t{hstate.bms.MCU_NTC[0]} °C, {hstate.bms.MCU_NTC[1]}°C')
        print(f'FootForce:\t\t{hstate.footForce}')
        print(f'FootForceEst:\t\t{hstate.footForceEst}')
        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')


    # Control commands here
    
    if i == 0:
        hcmd.mode = MotorModeHigh.FORCE_STAND
    elif i == 11:
        hcmd.mode = MotorModeHigh.DAMPING
    else:
        if (i % 2) == 0:
            hcmd.mode = MotorModeHigh.STAND_DOWN
        else:
            hcmd.mode = MotorModeHigh.STAND_UP
        # hcmd.mode = MotorModeHigh.STAND_UP
        # hcmd.mode = MotorModeHigh.STAND_DOWN



    cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
    conn.send(cmd_bytes)                    # Send the command
    time.sleep(1)                           # Some time to collect packets ;)

    i += 1                                  # Increment counter and break connection at end
    if i > 11:
        break
