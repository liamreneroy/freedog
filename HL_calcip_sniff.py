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

data = conn.getData()

paket_count = 0
for paket in data:
    hstate.parseData(paket)
    if paket_count == 0:
        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
        print(f'SN [{byte_print(hstate.SN)}]:\t{decode_sn(hstate.SN)}')
        print(f'Ver [{byte_print(hstate.version)}]:\t{decode_version(hstate.version)}')
        print(f'SOC:\t\t\t{hstate.bms.SOC} %')
        print(f'Overall Voltage:\t{getVoltage(hstate.bms.cell_vol)} mv') #something is still wrong here ?!
        print(f'Current:\t\t{hstate.bms.current} mA')
        print(f'Cycles:\t\t\t{hstate.bms.cycle}')
        print(f'Temps BQ:\t\t{hstate.bms.BQ_NTC[0]} °C, {hstate.bms.BQ_NTC[1]}°C')
        print(f'Temps MCU:\t\t{hstate.bms.MCU_NTC[0]} °C, {hstate.bms.MCU_NTC[1]}°C')
        print(f'FootForce:\t\t{hstate.footForce}')
        print(f'FootForceEst:\t\t{hstate.footForceEst}')
        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
        print()
        print()
    paket_count += 1

time.sleep(2)                                 
print("Initiating Control Loop...") # Some time to collect packets ;)
time.sleep(1)                                 
print()


motiontime = 0

# Initializaion
print(f"MotionTime: null")
print("Executing: RECOVERY\n")
hcmd.mode = MotorModeHigh.RECOVERY
cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
conn.send(cmd_bytes)                    # Send the command
time.sleep(0.2)

print(f"MotionTime: 0")
print("Executing: FORCE_STAND\n")
hcmd.mode = MotorModeHigh.FORCE_STAND
hcmd.euler = [0, 0, 0]
hcmd.bodyHeight = 0.0
cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
conn.send(cmd_bytes)                    # Send the command
time.sleep(0.2)


while True:
    
    motiontime += 1         # Increment motion time counter
    time.sleep(0.002)       # Sleep to achieve desired time interval (in seconds)

    if motiontime % 10 == 0: #Print every 10 cycles
        print(f"MotionTime: {motiontime}\n")
    
    data = conn.getData()
    for paket in data:
        hstate.parseData(paket)

    # Control commands here
    if(motiontime >= 0 and motiontime < 1000):
        hcmd.mode = MotorModeHigh.IDLE

    
    if(motiontime >= 1000 and motiontime < 2000):
        hcmd.mode = MotorModeHigh.FORCE_STAND
        hcmd.euler = [0, 0.2, 0]


    if(motiontime >= 3000 and motiontime < 4000):
        hcmd.mode = MotorModeHigh.FORCE_STAND
        hcmd.euler = [0, -0.3, 0]


    if(motiontime >= 4000 and motiontime < 6000):
        hcmd.mode = MotorModeHigh.FORCE_STAND
        hcmd.euler = [0, 0, 0]


    cmd_bytes = hcmd.buildCmd(debug=False)      # Build the command
    conn.send(cmd_bytes)                        # Send the command


    if motiontime >= 6000:
        print("Executing: STAND_DOWN")
        hcmd.mode = MotorModeHigh.STAND_DOWN
        cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
        conn.send(cmd_bytes)                    # Send the command
        time.sleep(1)

        print("Executing: DAMPING")
        hcmd.mode = MotorModeHigh.IDLE
        cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
        conn.send(cmd_bytes)                    # Send the command
        time.sleep(1)
        break                                   # Break connection at end








#####################
###  EXAMPLE CODE ###
#####################



### Example rotate 90° left
# hcmd.mode = MotorModeHigh.VEL_WALK
# hcmd.gaitType = GaitType.TROT
# hcmd.velocity = [0.04, 0.1]
# hcmd.yawSpeed = 2
# hcmd.footRaiseHeight = 0.1



### Example Pushups
# i = 0
# while True:
#     if i == 0:
#         hcmd.mode = MotorModeHigh.FORCE_STAND
#     elif i == 11:
#         hcmd.mode = MotorModeHigh.DAMPING
#     else:
#         if (i % 2) == 0:
#             hcmd.mode = MotorModeHigh.STAND_DOWN
#         else:
#             hcmd.mode = MotorModeHigh.STAND_UP
    

#     cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
#     conn.send(cmd_bytes)                    # Send the command
#     time.sleep(1)                           # Some time to collect packets ;)

#     i += 1                                  # Increment counter and break connection at end
#     if i > 11:
#         break




### Example walk
# motiontime = 0
# while True:
#     motiontime += 1
#     time.sleep(0.002)

#     if(motiontime > 0 and motiontime < 1000):
#         hcmd.mode = MotorModeHigh.FORCE_STAND
#         hcmd.euler = [-0.3, 0, 0]

#     if(motiontime > 1000 and motiontime < 2000):
#         hcmd.mode = MotorModeHigh.FORCE_STAND
#         hcmd.euler = [0.3, 0, 0]

#     if(motiontime > 2000 and motiontime < 3000):
#         hcmd.mode = MotorModeHigh.FORCE_STAND
#         hcmd.euler = [0, -0.2, 0]

#     if(motiontime > 3000 and motiontime < 4000):
#         hcmd.mode = MotorModeHigh.FORCE_STAND
#         hcmd.euler = [0, 0.2, 0]

#     if(motiontime > 4000 and motiontime < 5000):
#         hcmd.mode = MotorModeHigh.FORCE_STAND
#         hcmd.euler = [0, 0, -0.2]

#     if(motiontime > 5000 and motiontime < 6000):
#         hcmd.mode = MotorModeHigh.FORCE_STAND
#         hcmd.euler = [0.2, 0, 0]

#     if(motiontime > 6000 and motiontime < 7000):
#         hcmd.mode = MotorModeHigh.FORCE_STAND
#         hcmd.bodyHeight = -0.2

#     if(motiontime > 7000 and motiontime < 8000):
#         hcmd.mode = MotorModeHigh.FORCE_STAND
#         hcmd.bodyHeight = 0.1

#     if(motiontime > 8000 and motiontime < 9000):
#         hcmd.mode = MotorModeHigh.FORCE_STAND
#         hcmd.bodyHeight = 0.0

#     if(motiontime > 9000 and motiontime < 11000):
#         hcmd.mode = MotorModeHigh.STAND_DOWN

#     if(motiontime > 11000 and motiontime < 13000):
#         hcmd.mode = MotorModeHigh.STAND_UP

#     if(motiontime > 13000 and motiontime < 14000):
#         hcmd.mode = MotorModeHigh.Idle

#     if(motiontime > 14000 and motiontime < 18000):
#         hcmd.mode = MotorModeHigh.VEL_WALK
#         hcmd.gaitType = GaitType.TROT
#         hcmd.velocity = [0.4, 0] # -1  ~ +1
#         hcmd.yawSpeed = SpeedLevel.HIGH_SPEED
#         hcmd.footRaiseHeight = 0.1
#         # printf("walk\n")

#     if(motiontime > 18000 and motiontime < 20000):
#         hcmd.mode = MotorModeHigh.Idle
#         hcmd.velocity = [0, 0]

#     if(motiontime > 20000 and motiontime < 24000):
#         hcmd.mode = MotorModeHigh.VEL_WALK
#         hcmd.gaitType = GaitType.TROT
#         hcmd.velocity = [0.2, 0] # -1  ~ +1
#         hcmd.bodyHeight = 0.1

#     cmd_bytes = hcmd.buildCmd(debug=False)
#     conn.send(cmd_bytes)

#     if motiontime > 24000:
#         break
# # time.sleep(0.1) - not sure if this is necessary (originally commented out)



