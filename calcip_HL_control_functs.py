from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType, SpeedLevel
from ucl.complex import motorCmd, led
import time
import math


# You can use one of the 3 Presets WIFI_DEFAULTS, LOW_CMD_DEFAULTS or HIGH_CMD_DEFAULTS.
# IF NONE OF THEM ARE WORKING YOU CAN DEFINE A CUSTOM ONE LIKE THIS:
#
# MY_CONNECTION_SETTINGS = (listenPort, addr_wifi, sendPort_high, local_ip_wifi)
# conn = unitreeConnection(MY_CONNECTION_SETTINGS)


def roll_shake(publish_hz=250, granularity_override=None, amplitude=0.7, frequency=1, reversed=False):
    '''ARG RANGES:
    publish_hz = [50, 500] (Hz) -> smaller equals faster execution time. If robot acting weird, try decreasing this value. Recommended 100+ 
    
    granularity_override = [100, 4000] (cycles) -> smaller equals slightly faster but choppier execution. Recommended 500+ for smooth motion.
    
    amplitude = [0, 0.8] (radians) -> smaller equals less roll angle. Recommended 0.6 for full roll angle.
    
    frequency = [1, 3] (cycles) -> quantity equals number of oscillations. Recommended 1 for one oscillation. 
                                   Value over 1 may trigger an override of publish_hz and granularity.
    
    reversed = [True, False] -> True reverses the direction of the roll_shake
    
    
    Note: You want to proportionally set your publishing_hz and granularity. 
          If publish_hz is high [250], granularity should likely be set to 2-3x that value [500-750]. 
    '''

    granularity_multiplier = 3   # Either 2x or 3x publish_hz is recommended

    # Check if frequency is valid
    if isinstance(frequency, int) == False or frequency > 3:
        raise ValueError("frequency must be integer and within range [1, 3]")

    # Check if granularity_override is valid
    if granularity_override % 10 != 0 or granularity > 4000 or granularity < 100:
        raise ValueError("granularity_override must be integer, divisible by 10 and within range [100, 4000]")
    
    # Override frequency and granularity if frequency/publishing_hz is too high
    if frequency >= 2 and publish_hz > 250:
        publish_hz = 250
        granularity_override = publish_hz * granularity_multiplier
        print(f"WARNING: Combination of frequency and publish_hz to high -> Overriding publish_hz to {publish_hz}")

    # Override granularity if granularity_override is set
    if granularity_override == None:
        granularity = publish_hz * granularity_multiplier
    else:
        granularity = granularity_override
        print(f"WARNING: Overriding granularity to {granularity_override} [recommended 2-3x publish_hz]")

    # Check if publish_hz is valid
    if isinstance(publish_hz, int) == False or publish_hz > 500 or publish_hz < 50:
        raise ValueError("publish_hz must be integer and within range [50, 500]")
    
    # Check if granularity is valid
    if granularity % 10 != 0 or granularity > 4000 or granularity < 100:
        raise ValueError("granularity must be integer, divisible by 10 and within range [100, 4000]")
    
    # Check if amplitude is valid
    if isinstance(amplitude, float) == False or amplitude > 0.8 or amplitude < 0:
        raise ValueError("amplitude must be float and within range [0, 0.8]")

    # Reverse the direction of the roll if set to True
    if reversed == True: 
        amplitude = -amplitude


    print('roll_shake ::: Publishing at {}Hz\n'.format(publish_hz))

    for timestep in range(0, granularity+1):    # Not sure if I need the +1 but this makes it start/end at zero
        time.sleep(float(1./publish_hz))        # Average  --> Sleep to achieve desired 0.002 time interval (in seconds)

        # Set highCmd values 
        hcmd.mode = MotorModeHigh.FORCE_STAND   
        hcmd.euler = [amplitude * math.sin(frequency * timestep / granularity * ((math.pi)*2)), 0, 0]

        if timestep % 10 == 0:                  #Print every 10 cycles
            print(f"TimeStep: {timestep} ,  Roll Angle: {hcmd.euler}\n")

        cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
        conn.send(cmd_bytes)                    # Send the command


















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


motiontime = 0      # Initializaion

print(f"MotionTime: null")              # Allows robo to start from any state: IDLE / FORCE_STAND / DAMPED
print("Executing: RECOVERY\n")    
hcmd.mode = MotorModeHigh.RECOVERY
cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
conn.send(cmd_bytes)                    # Send the command
time.sleep(1)                           # Sleep for 1 second


print(f"MotionTime: 0")                 # Put robot in into standing state to start
print("Executing: FORCE_STAND\n")
hcmd.mode = MotorModeHigh.FORCE_STAND
hcmd.euler = [0, 0, 0]
hcmd.bodyHeight = 0.0
cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
conn.send(cmd_bytes)                    # Send the command
time.sleep(1)                           # Sleep for 1 second


while True:
    
    data = conn.getData()
    for paket in data:
        hstate.parseData(paket)

    # Control commands here
    roll_shake()

    time.sleep(1)

    roll_shake(frequency=2)

    time.sleep(1)

    roll_shake()

    print("Executing: STAND_DOWN")
    hcmd.mode = MotorModeHigh.STAND_DOWN
    cmd_bytes = hcmd.buildCmd(debug=False)  # Build the lay-down command
    conn.send(cmd_bytes)                    # Send the lay-down command
    time.sleep(1)                           # Sleep for 1 second

    print("Executing: IDLE")
    hcmd.mode = MotorModeHigh.IDLE
    cmd_bytes = hcmd.buildCmd(debug=False)  # Build the idle command
    conn.send(cmd_bytes)                    # Send the idle command
    time.sleep(0.5)                         # Sleep for 0.5 second

    print("Executing: DAMPING")
    hcmd.mode = MotorModeHigh.DAMPING
    cmd_bytes = hcmd.buildCmd(debug=False)  # Build the final command
    conn.send(cmd_bytes)                    # Send the final command
    time.sleep(1)                           # Sleep for 1 second
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



