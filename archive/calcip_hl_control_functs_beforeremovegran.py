from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType, SpeedLevel
from ucl.complex import motorCmd, led

import time
import datetime

import math

import numpy as np
np.set_printoptions(formatter={'float': lambda x: '%7.4f' % (x)})
# np.set_printoptions(precision=3)



def initialize_control():
    
    '''
    You can use one of the 3 Presets WIFI_DEFAULTS, LOW_CMD_DEFAULTS or HIGH_CMD_DEFAULTS.
    IF NONE OF THEM ARE WORKING YOU CAN DEFINE A CUSTOM ONE LIKE THIS:

    MY_CONNECTION_SETTINGS = (listenPort, addr_wifi, sendPort_high, local_ip_wifi)
    conn = unitreeConnection(MY_CONNECTION_SETTINGS)
    '''

    conn = unitreeConnection(HIGH_WIFI_DEFAULTS)    # Creates a new connection object with HIGH_WIFI_DEFAULTS named 'conn'
    conn.startRecv()                                # Starts up connection 
    hcmd = highCmd()                                # Creates the highCmd object named hcmd
    hstate = highState()      

    cmd_bytes = hcmd.buildCmd(debug=False)          # Builds an empty command
    conn.send(cmd_bytes)                            # Send empty command to tell the dog the receive port and initialize the connection
    time.sleep(0.5)                                 # Some time to collect packets ;)

    print(">> Control Loop Initialized\n")        

    return conn, hcmd, hstate                       # Return connection, highCmd and highState objects


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def parse_data(conn, hstate, print_out=False):

    print(">> Parsing Data\n")        

    data = conn.getData()

    paket_count = 0
    for paket in data:
        hstate.parseData(paket)

        if print_out == True and paket_count == 0:
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


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def recover_control(conn, hcmd):

    print(">> Executing: RECOVERY\n")          # Allows robo to start from any state: IDLE / FORCE_STAND / DAMPED
    hcmd.mode = MotorModeHigh.RECOVERY
    cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
    conn.send(cmd_bytes)                    # Send the command
    time.sleep(1)                           # Sleep for 1 second

    print(">> Executing: FORCE_STAND\n")       # Put robot in into standing state to start
    hcmd.mode = MotorModeHigh.FORCE_STAND
    hcmd.euler = [0, 0, 0]
    hcmd.bodyHeight = 0.0
    cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
    conn.send(cmd_bytes)                    # Send the command
    time.sleep(1)                           # Sleep for 1 second


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def terminate_control(conn, hcmd):

    print(">> Executing: STAND_DOWN\n")
    hcmd.mode = MotorModeHigh.STAND_DOWN
    cmd_bytes = hcmd.buildCmd(debug=False)  # Build the lay-down command
    conn.send(cmd_bytes)                    # Send the lay-down command
    time.sleep(1)                           # Sleep for 1 second

    print(">> Executing: IDLE\n")
    hcmd.mode = MotorModeHigh.IDLE
    cmd_bytes = hcmd.buildCmd(debug=False)  # Build the idle command
    conn.send(cmd_bytes)                    # Send the idle command
    time.sleep(0.5)                         # Sleep for 0.5 second

    print(">> Executing: DAMPING\n")
    hcmd.mode = MotorModeHigh.DAMPING
    cmd_bytes = hcmd.buildCmd(debug=False)  # Build the final command
    conn.send(cmd_bytes)                    # Send the final command
    time.sleep(1)                           # Sleep for 1 second
    
    return True                             # Break connection at end

              
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


# def sin_roll(conn, hcmd, publish_hz=250, granularity_override=None, amplitude=0.6, frequency=1, reversed=False):
#     '''ARG RANGES:
#     publish_hz = [50, 500] (Hz) -> smaller equals faster execution time. If robot acting weird, try decreasing this value. Recommended 100+ 
    
#     granularity_override = [100, 4000] (number of cycles) -> smaller equals slightly faster but choppier execution. Recommended 500+ for smooth motion.
    
#     amplitude = [0, 0.8] (radians) -> smaller equals less roll angle. Recommended 0.6 for full roll angle.
    
#     frequency = [1, 3] (cycles) -> quantity equals number of oscillations. Recommended 1 for one oscillation. 
#                                    Value over 1 may trigger an override of publish_hz and granularity.
    
#     reversed = [True, False] -> True reverses the direction of the sin_roll
    
    
#     Note: You want to proportionally set your publishing_hz and granularity. 
#           If publish_hz is high [250], granularity should likely be set to 2-3x that value [500-750]. 
#     '''

#     granularity_multiplier = 3   # Either 2x or 3x publish_hz is recommended

#     # Check if frequency is valid
#     if isinstance(frequency, int) == False or frequency > 3:
#         raise ValueError("frequency must be integer and within range [1, 3]")

#     if granularity_override:
#         # Check if granularity_override is valid
#         if granularity_override % 10 != 0 or granularity > 4000 or granularity < 100:
#             raise ValueError("granularity_override must be integer, divisible by 10 and within range [100, 4000]")
        
#     # Override frequency and granularity if frequency/publishing_hz is too high
#     if frequency >= 2 and publish_hz > 250:
#         publish_hz = 250
#         granularity_override = publish_hz * granularity_multiplier
#         print(f"WARNING: Combination of frequency and publish_hz to high -> Overriding publish_hz to {publish_hz}")

#     # Override granularity if granularity_override is set
#     if granularity_override == None:
#         granularity = publish_hz * granularity_multiplier
#     else:
#         granularity = granularity_override
#         print(f"WARNING: Overriding granularity to {granularity_override} [recommended 2-3x publish_hz]")

#     # Check if publish_hz is valid
#     if isinstance(publish_hz, int) == False or publish_hz > 500 or publish_hz < 50:
#         raise ValueError("publish_hz must be integer and within range [50, 500]")
    
#     # Check if granularity is valid
#     if granularity % 10 != 0 or granularity > 4000 or granularity < 100:
#         raise ValueError("granularity must be integer, divisible by 10 and within range [100, 4000]")
    
#     # Check if amplitude is valid
#     if isinstance(amplitude, float) == False or amplitude > 0.8 or amplitude < 0:
#         raise ValueError("amplitude must be float and within range [0, 0.8]")

#     # Reverse the direction of the roll if set to True
#     if reversed == True: 
#         amplitude = -amplitude

#     print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
#     print('>> Executing: sin_roll ::: Publishing at {}Hz\n'.format(publish_hz))
#     print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')

#     for timestep in range(0, granularity+1):    # Not sure if I need the +1 but this makes it start/end at zero
#         time.sleep(float(1./publish_hz))        # Average  --> Sleep to achieve desired 0.002 time interval (in seconds)

#         # Set highCmd values 
#         hcmd.mode = MotorModeHigh.FORCE_STAND   
#         hcmd.euler = [amplitude * math.sin(frequency * timestep / granularity * ((math.pi)*2)), 0, 0]

#         if timestep % 10 == 0:                  #Print every 10 cycles
#             print(f"TimeStep: {timestep} ,  Roll Angle: {hcmd.euler}\n")

#         cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
#         conn.send(cmd_bytes)                    # Send the command


# # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


# def sin_pitch(conn, hcmd, publish_hz=250, granularity_override=None, amplitude=0.6, frequency=1, reversed=False):
#     '''ARG RANGES:
#     publish_hz = [50, 500] (Hz) -> smaller equals faster execution time. If robot acting weird, try decreasing this value. Recommended 100+ 
    
#     granularity_override = [100, 4000] (number of cycles) -> smaller equals slightly faster but choppier execution. Recommended 500+ for smooth motion.
    
#     amplitude = [0, 0.8] (radians) -> smaller equals less pitch angle. Recommended 0.6 for full pitch angle.
    
#     frequency = [1, 3] (cycles) -> quantity equals number of oscillations. Recommended 1 for one oscillation. 
#                                    Value over 1 may trigger an override of publish_hz and granularity.
    
#     reversed = [True, False] -> True reverses the direction of the sin_pitch
    
    
#     Note: You want to proportionally set your publishing_hz and granularity. 
#           If publish_hz is high [250], granularity should likely be set to 2-3x that value [500-750]. 
#     '''

#     granularity_multiplier = 3   # Either 2x or 3x publish_hz is recommended

#     # Check if frequency is valid
#     if isinstance(frequency, int) == False or frequency > 3:
#         raise ValueError("frequency must be integer and within range [1, 3]")

#     if granularity_override:
#         # Check if granularity_override is valid
#         if granularity_override % 10 != 0 or granularity > 4000 or granularity < 100:
#             raise ValueError("granularity_override must be integer, divisible by 10 and within range [100, 4000]")
        
#     # Override frequency and granularity if frequency/publishing_hz is too high
#     if frequency >= 2 and publish_hz > 250:
#         publish_hz = 250
#         granularity_override = publish_hz * granularity_multiplier
#         print(f"WARNING: Combination of frequency and publish_hz to high -> Overriding publish_hz to {publish_hz}")

#     # Override granularity if granularity_override is set
#     if granularity_override == None:
#         granularity = publish_hz * granularity_multiplier
#     else:
#         granularity = granularity_override
#         print(f"WARNING: Overriding granularity to {granularity_override} [recommended 2-3x publish_hz]")

#     # Check if publish_hz is valid
#     if isinstance(publish_hz, int) == False or publish_hz > 500 or publish_hz < 50:
#         raise ValueError("publish_hz must be integer and within range [50, 500]")
    
#     # Check if granularity is valid
#     if granularity % 10 != 0 or granularity > 4000 or granularity < 100:
#         raise ValueError("granularity must be integer, divisible by 10 and within range [100, 4000]")
    
#     # Check if amplitude is valid
#     if isinstance(amplitude, float) == False or amplitude > 0.8 or amplitude < 0:
#         raise ValueError("amplitude must be float and within range [0, 0.8]")

#     # Reverse the direction of the pitch if set to True
#     if reversed == True: 
#         amplitude = -amplitude

#     print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
#     print('>> Executing: sin_pitch ::: Publishing at {}Hz\n'.format(publish_hz))
#     print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')

#     for timestep in range(0, granularity+1):    # Not sure if I need the +1 but this makes it start/end at zero
#         time.sleep(float(1./publish_hz))        # Average  --> Sleep to achieve desired 0.002 time interval (in seconds)

#         # Set highCmd values 
#         hcmd.mode = MotorModeHigh.FORCE_STAND   
#         hcmd.euler = [0, amplitude * math.sin(frequency * timestep / granularity * ((math.pi)*2)), 0]

#         if timestep % 10 == 0:                  #Print every 10 cycles
#             print(f"TimeStep: {timestep} ,  Pitch Angle: {hcmd.euler}\n")

#         cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
#         conn.send(cmd_bytes)                    # Send the command


# # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



# def sin_yaw(conn, hcmd, publish_hz=250, granularity_override=None, amplitude=0.5, frequency=1, reversed=False):
#     '''ARG RANGES:
#     publish_hz = [50, 500] (Hz) -> smaller equals faster execution time. If robot acting weird, try decreasing this value. Recommended 100+ 
    
#     granularity_override = [100, 4000] (number of cycles) -> smaller equals slightly faster but choppier execution. Recommended 500+ for smooth motion.
    
#     amplitude = [0, 0.8] (radians) -> smaller equals less yaw angle. Recommended 0.6 for full yaw angle.
    
#     frequency = [1, 3] (cycles) -> quantity equals number of oscillations. Recommended 1 for one oscillation. 
#                                    Value over 1 may trigger an override of publish_hz and granularity.
    
#     reversed = [True, False] -> True reverses the direction of the sin_yaw
    
    
#     Note: You want to proportionally set your publishing_hz and granularity. 
#           If publish_hz is high [250], granularity should likely be set to 2-3x that value [500-750]. 
#     '''

#     granularity_multiplier = 3   # Either 2x or 3x publish_hz is recommended

#     # Check if frequency is valid
#     if isinstance(frequency, int) == False or frequency > 3:
#         raise ValueError("frequency must be integer and within range [1, 3]")

#     if granularity_override:
#         # Check if granularity_override is valid
#         if granularity_override % 10 != 0 or granularity > 4000 or granularity < 100:
#             raise ValueError("granularity_override must be integer, divisible by 10 and within range [100, 4000]")
        
#     # Override frequency and granularity if frequency/publishing_hz is too high
#     if frequency >= 2 and publish_hz > 250:
#         publish_hz = 250
#         granularity_override = publish_hz * granularity_multiplier
#         print(f"WARNING: Combination of frequency and publish_hz to high -> Overriding publish_hz to {publish_hz}")

#     # Override granularity if granularity_override is set
#     if granularity_override == None:
#         granularity = publish_hz * granularity_multiplier
#     else:
#         granularity = granularity_override
#         print(f"WARNING: Overriding granularity to {granularity_override} [recommended 2-3x publish_hz]")

#     # Check if publish_hz is valid
#     if isinstance(publish_hz, int) == False or publish_hz > 500 or publish_hz < 50:
#         raise ValueError("publish_hz must be integer and within range [50, 500]")
    
#     # Check if granularity is valid
#     if granularity % 10 != 0 or granularity > 4000 or granularity < 100:
#         raise ValueError("granularity must be integer, divisible by 10 and within range [100, 4000]")
    
#     # Check if amplitude is valid
#     if isinstance(amplitude, float) == False or amplitude > 0.8 or amplitude < 0:
#         raise ValueError("amplitude must be float and within range [0, 0.8]")

#     # Reverse the direction of the yaw if set to True
#     if reversed == True: 
#         amplitude = -amplitude

#     print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
#     print('>> Executing: sin_yaw ::: Publishing at {}Hz\n'.format(publish_hz))
#     print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')

#     for timestep in range(0, granularity+1):    # Not sure if I need the +1 but this makes it start/end at zero
#         time.sleep(float(1./publish_hz))        # Average  --> Sleep to achieve desired 0.002 time interval (in seconds)

#         # Set highCmd values 
#         hcmd.mode = MotorModeHigh.FORCE_STAND   
#         hcmd.euler = [0, 0, amplitude * math.sin(frequency * timestep / granularity * ((math.pi)*2))]

#         if timestep % 10 == 0:                  #Print every 10 cycles
#             print(f"TimeStep: {timestep} ,  Yaw Angle: {hcmd.euler}\n")

#         cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
#         conn.send(cmd_bytes)                    # Send the command


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def sin_rollpitchyaw(conn, hcmd, publish_hz=200, granularity_multiplier=1., sleep_override=None, loop_repeats=1, 
                     rollpitchyaw_array=np.array([False, False, False]),
                     reversed_array=np.array([1, 1, 1]),
                     amplitude_array=np.array([0.5, 0.5, 0.5]),
                     offset_array=np.array([0, 0, 0]), 
                     period_array=np.array([1, 1, 1]), 
                     phase_array=np.array([0, 0, 0])):
    
    '''ARG RANGES:
    recall y = Asin(B(x + C)) + D   ///   A = amplitude, 2pi/B = period, C = phase shift, D = offset

    publish_hz = [50, 500] (Hz - integer) -> Smaller equals less msgs being sent per loop. Smoother motion at higher 
                                             Hz but more comp expensive. If robot acting weird, decreasing this value. Recommended 100+ 
    
                                             
    granularity_multiplier = [0.2, 4.0] (float) -> Resulting granularity is: publish_hz * granularity_multiplier
                                                   If loop rate=1 is held [1 loop = 1sec], then granularity is number of
                                                   messages that are being published per second. Recommended 1.0. 

    # EDIT
    sleep_override = [0.001, 0.1] (integer) ->  Sets the number of seconds that each loop will sleep for. Setting this breaks the loop_rate=1 [1 loop = 1sec]
                                                Smaller equals faster loops, but more comp expensive. If you have high publish_hz [200+] and large 
                                                granularity_multiplier [2+], then you can use this to slow down the loops to get smoother motion.
    
    loop_repeats = [1, 100] (integer) -> number of times to repeat the loop. Recommended 1 for one loop.

    rollpitchyaw_array = [bool] -> True enables the sin_roll, sin_pitch, sin_yaw respectively. False disables the sin_roll, sin_pitch, sin_yaw respectively.

    reversed_array = [1, -1] -> 1 keeps direction same, while -1 reverses the sin_roll, sin_pitch, sin_yaw respectively.

    amplitude_array = [-0.7, 0.7] (radians) -> Larger value equals bigger angle. Negative is reversed. Recommended 0.6 for full angle.
                                               CAREFUL WITH THIS ONE.. dont max joint limits
    
    offset_array = [-0.7, 0.7] (radians) -> Offsets the oscillation. Combination of offset and amplitude cannot exceed 0.7.
                                         CAREFUL WITH THIS ONE.. dont max joint limits.

    period_array = [1./3, 10] (cycles) -> quantity equals period of oscillation. Recommended 1 for one oscillation. 2 for half oscillation.
                                          value 1./3 completes three oscillations. Less than 1./3 may trigger an override of publish_hz and granularity.
    
    phase_array = [-2pi, 2pi] (radians) -> Use math.pi // quantity equals phase shift of oscillation. Recommended 0 for no phase shift. math.pi for half phase shift.              

    Note:   You want a loop_rate of 1 as [1 loop = 1sec]
            The following equation is used to calculate loop_rate: 
            
            sleep_rate = 1 / (publish_hz * granularity_multiplier) or (1 / granularity)

            loop_rate = sleep_rate * granularity

            This loop_rate will keep to 1 so long as granularity_override is not used.
            
            # EDIT
            Granularity is inversely proportional to sleep_multiplier.
            Set your publishing_hz and sleep rate as necessary to achieve a loop rate of 1. 
            Granularity is set to be (publishing_hz / sleep_multiplier) unless an override value is specified.

            An easy way to increase/decrease computation is adjust publish_hz while keeping sleep_multiplier=1 constant.
            You can override granularity to get smoother motion while breaking loop rate with granularity_override.  
    '''

    # Check if granularity_multiplier is valid
    if granularity_multiplier > 4.0 or granularity_multiplier < 0.2:
        raise ValueError("granularity_multiplier must be within range [0.2, 4]")

    # Check if granularity_override exists and is valid
    if sleep_override:
        if sleep_override > 0.1 or sleep_override < 0.001:
            raise ValueError("sleep_override must be within range [0.001, 0.1]")
        
    # Check if period is valid
    for period in period_array:
        if period > 10 or period < 1./3:
            raise ValueError("period values must be within range [1/3, 10] for each euler angle")



# EDIT add sleep entry here to fortify this check
    # Override period and granularity if period/publish_hz is too high 
    for period in period_array:
        if period < 0.5 and publish_hz > 250 and granularity_multiplier > 1.5:
            print(f"WARNING: Combination of low period {period} and high granularity {granularity} is beyond safety limiter")
            publish_hz = 250
            granularity_multiplier = 1.5
            print(f"::: Overriding publish_hz to: {publish_hz}") 
            print(f"::: Overriding granularity_multiplier to: {granularity_multiplier}") 

    # Set granularity and calculate sleep_rate
    if publish_hz * granularity_multiplier % 1 != 0:
        raise ValueError("publish_hz * granularity_multiplier must equal an integer")
    
    granularity = int(publish_hz * granularity_multiplier)
    sleep_rate = 1 / (publish_hz * granularity_multiplier)

    # Override granularity if granularity_override is set
    if sleep_override:
        sleep_rate = sleep_override
        print(f"WARNING: Overriding sleep_rate to {sleep_override} [breaking loop_rate=1]")


    # Check if sleep_rate is valid
    if sleep_rate < 0.002:
        print(f"WARNING: low sleep_rate of {sleep_rate} is not recommended. Might experience control lag") 
        print(f"::: Recommended sleep_rate >= 0.002. Adjust publish_hz and granularity_multiplier to achieve this")
        if sleep_rate < 0.001:
            raise ValueError("sleep_rate must be within range [1/3, 10] for each euler angle")

    # Check if publish_hz is valid
    if isinstance(publish_hz, int) == False or publish_hz > 500 or publish_hz < 50:
        raise ValueError("publish_hz must be integer and within range [50, 500]")
    
    # Check if granularity is valid
    if granularity % 10 != 0 or granularity > 4000 or granularity < 100:
        raise ValueError("granularity must be integer, divisible by 10 and within range [100, 4000]")
    
    # Check if amplitude + offset is valid
    for index_123 in range(0,3):
        if abs(amplitude_array[index_123]) + abs(offset_array[index_123]) > 0.7:
            raise ValueError("amplitude + offet must be within range [0, 0.7] for each euler angle")

    # Check if phase is valid
    for phase in phase_array:
        if phase > 2*math.pi or phase < -2*math.pi:
            print(f'WARNING: phase values should be specified within range [-2pi, 2pi] for each euler angle')

    # Calculate loop_rate (should be 1 if sleep_override is not used)
    # loop_rate = (publish_hz * granularity_multiplier) / granularity (old)
    loop_rate = sleep_rate * granularity

    # EDIT and also add an input() for first
    print('\n+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
    print('>> Executing: sin_rollpitchyaw')
    print(f'::: Publish Rate: {publish_hz}Hz \t(How often to send each TimeStep)')
    print(f'::: Granular Multip {granularity_multiplier} \t(Multiply by publish_hz to get granularity)')
    print(f'::: Granularity: {granularity} \t\t(Steps per one loop)')
    print(f'::: Sleep Rate: {sleep_rate} \t\t(How long to sleep each loop)')
    print(f'RESULTING LOOP RATE: {loop_rate} --> [approx. {loop_rate} loop / 1 second]\n')
    if loop_rate != 1:
        print(f'WARNING: Loop rate is not 1 aka [1 loop=1sec]')
        print('::: Adjust sleep_override to achieve desired loop rate of 1')
        print('::: Increase sleep_override to lower loop rate /// Decrease sleep_override to increase loop rate\n')
        # EDIT: Could calculate what they need it to be and tell them to adjust sleep_override
    print(f'::: RollPitchYaw: {rollpitchyaw_array}')
    print(f'::: Amplitude: {amplitude_array}')
    print(f'::: Period: {period_array}')
    print(f'::: Phase Shift: {phase_array}')
    print(f'::: Reversed: {reversed_array}\n')
    print(f'::: Offset: {offset_array}')
    print(f'::: Repeat Loop: {loop_repeats} times\n')
    print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')


    a = datetime.datetime.now()
    for timestep in range(0, granularity*loop_repeats):     # Not sure if I need the +1 but this makes it start/end at zero

        # Set highCmd values 
        hcmd.mode = MotorModeHigh.FORCE_STAND   
        hcmd.euler = np.multiply(np.array([
            (reversed_array[0] * amplitude_array[0]) * math.sin(((math.pi)*2 / period_array[0]) * ((timestep / granularity) + phase_array[0])) + offset_array[0],
            (reversed_array[1] * amplitude_array[1]) * math.sin(((math.pi)*2 / period_array[1]) * ((timestep / granularity) + phase_array[1])) + offset_array[1],
            (reversed_array[2] * amplitude_array[2]) * math.sin(((math.pi)*2 / period_array[2]) * ((timestep / granularity) + phase_array[2])) + offset_array[2]]),
            rollpitchyaw_array)
        
        if timestep % 10 == 0:                  # Print every 10 cycles
            if timestep % granularity == 0:     # Print once per loop to check loop rate
                b = datetime.datetime.now()     # Time (ignore first entry)
                c = b - a
                a = datetime.datetime.now()
                print(f'TimeStep: {timestep:04d} \t  Euler RPY Angle: {hcmd.euler} \t LoopRate: {c.total_seconds()} secs\n')
                continue
            
            print(f'TimeStep: {timestep:04d} \t  Euler RPY Angle: {hcmd.euler}\n')

        cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
        conn.send(cmd_bytes)                    # Send the command

        time.sleep(float(sleep_rate))           # Average  --> Pub Rate: Shouldn't dip below 0.002 - test w/ MacBook for proper sleeptime 

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Then add offset and phase shift to the sin function
# Add an absolute value shifter [-1, 0, 1] but this might make the motion choppy at point of shift
# Then add a function to change the period of the sin function
# Then add a function to change the amplitude of the sin function
# Then add a function to change the offset of the sin function
# Then add a function to change the phase shift of the sin function
# Then add a function to change the number of cycles of the sin function
# Add time printout to sin function