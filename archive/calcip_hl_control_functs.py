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

    print(">> Executing: FORCE_STAND\n")    # Put robot back into neutral standing state
    hcmd.mode = MotorModeHigh.FORCE_STAND
    hcmd.euler = [0, 0, 0]
    hcmd.bodyHeight = 0.0
    cmd_bytes = hcmd.buildCmd(debug=False)  # Build the stand command
    conn.send(cmd_bytes)                    # Send the stand command
    time.sleep(1)                           # Sleep for 1 second

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

def sin_rollpitchyaw(conn, hcmd, publish_hz=200, sleep_override=None, loop_repeats=2, 
                     rollpitchyaw_array=np.array([False, False, False]),
                     amplitude_array=np.array([0.5, 0.5, 0.4]),
                     offset_array=np.array([0, 0, 0]), 
                     period_array=np.array([1, 1, 1]), 
                     phase_array=np.array([0, 0, 0]), 
                     dev_check=True,
                     printer=True):
    
    '''ARG RANGES:
    recall y = Asin(B(x + C)) + D   ///   A = amplitude, 2pi/B = period, C = phase shift, D = offset

    publish_hz = [50, 1000] (Hz - integer) -> Number of msgs being sent per loop. Smoother motion at higher hz but more comp expensive. 
                                             If robot acting undesirably, try decreasing this value. Recommended 100+ 
    
    sleep_override = [0.001, 0.1] (integer) ->  Sets the number of seconds that each loop will sleep for. Setting this breaks the loop_rate=1 [1sec/loop]
                                                Smaller equals faster loops, but more comp expensive. If you have high publish_hz [200+], then you 
                                                can use this to slow down the loops to get smoother motion.
    
    loop_repeats = [1, 100] (integer) -> number of times to repeat the loop. Use 1 for one loop.

    rollpitchyaw_array = [bool] -> True enables the sin_roll, sin_pitch, sin_yaw respectively. False disables the sin_roll, sin_pitch, sin_yaw respectively.

    amplitude_array = [-0.7, 0.7] (radians) -> Larger value equals bigger angle. Negative is reversed. Recommended 0.6 for full angle.
                                               CAREFUL WITH THIS ONE.. dont max joint limits. Set skip_check=False first to check your code.
    
    offset_array = [-0.7, 0.7] (radians) -> Offsets the oscillation. Combination of offset and amplitude cannot exceed 0.7.
                                            CAREFUL WITH THIS ONE.. dont max joint limits. Set skip_check=False first to check your code.

    period_array = [1./3, 10] (cycles) -> quantity equals period of oscillation. Recommended 1 for one oscillation. 2 for half oscillation.
                                          Use 0.5 to get double the oscillations. Use 0.333 to get triple the oscillations. Use 0 to get infinite oscillations? (j.k. dont do that)
                                          Value less than 0.3333 will trigger an override of publish_hz so you dont explode robot.
    
    phase_array = [-2pi, 2pi] (radians) -> Use math.pi // quantity equals phase shift of oscillation. Recommended 0 for no phase shift. math.pi for half phase shift.              

    dev_check = [True, False] -> If True, will check if all values are within range. If False, will skip all checks.
    
    printer = [True, False] -> If True, will print out the values of the control loop. If False, will not print out the values of the control loop.

    Note:   If you want a loop_rate of 1 as [1sec/loop] -> this is useful for making robot move to a given time signature..
            The following equation is used to calculate loop_rate: 
            
            loop_rate = sleep_rate * publish_hz
            sleep_rate = 1 / publish_hz

            This loop_rate will keep to 1 so long as sleep_override is not used.
            You can increase publish_hz and override the sleep_rate to get smoother motion while breaking loop rate.  
    '''

    sleep_rate = 1 / int(publish_hz) # Calculate sleep_rate

    # Override sleep_rate if sleep_override is set
    if sleep_override:
        sleep_rate = sleep_override
        print(f"WARNING: Overriding sleep_rate to {sleep_override} [breaking loop_rate=1]")

    if dev_check: # Skip checks ONLY if you are very confident in your code >:) 

        # Check if granularity_override exists and is valid
        if sleep_override:
            if sleep_override > 0.1 or sleep_override < 0.001:
                raise ValueError("sleep_override must be within range [0.001, 0.1]")
            
        # Check if period is valid
        for period in period_array:
            if period > 10 or period < 1./3:
                raise ValueError("period values must be within range [1/3, 10] for each euler angle")

        # Check if period and publish_hz combo is beyond safety limiter and override publish_hz if necessary 
        for period in period_array:
            if period < 0.5 and publish_hz > 250:
                print(f"WARNING: Combination of low period {period} and publish_hz {publish_hz} is beyond safety limiter")
                publish_hz = 250
                sleep_rate = 1 / int(publish_hz)
                print(f"::: Overriding publish_hz to: {publish_hz}") 

        # Check if publish_hz is valid
        if isinstance(publish_hz, int) == False or publish_hz > 1000 or publish_hz < 50:
            raise ValueError("publish_hz must be integer and within range [50, 1000]")
        
        # Check if sleep_rate is valid
        if sleep_rate < 0.002:
            print(f"WARNING: low sleep_rate of {sleep_rate} is not recommended. Might experience control lag") 
            print(f"::: Recommended sleep_rate >= 0.002. Decrease publish_hz to 500 or less or override sleep_rate")
            if sleep_rate < 0.001:
                raise ValueError("sleep_rate must be within range [0.001, 0.1]")

        # Check if amplitude + offset is valid
        for index_123 in range(0,3):
            if abs(amplitude_array[index_123]) + abs(offset_array[index_123]) > 0.7:
                raise ValueError("sum of (amplitude + offset) must be within range [0, 0.7] for each euler angle")

        # Check if phase is valid
        for phase in phase_array:
            if phase > 2*math.pi or phase < -2*math.pi:
                print(f'WARNING: phase values should be specified within range [-2pi, 2pi] for each euler angle')

    # Calculate loop_rate (should be 1 if sleep_override is not used)
    loop_rate = sleep_rate * publish_hz

    if dev_check:
        print('\n+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
        print('>> Executing: sin_rollpitchyaw')
        print(f'::: Publish Rate: {publish_hz}Hz \t(How many messages per loop)')
        print(f'::: Sleep Rate: {sleep_rate} \t\t(How long to after each message)\n')
        print(f'RESULTING LOOP RATE: {loop_rate} \t(Approx. {loop_rate} second per loop)\n')
        if loop_rate != 1:
            print(f'WARNING: Loop rate is not 1 aka [not {loop_rate} second per loop]')
            print('::: This is likely due to sleep_override being used')
            print('::: Adjust sleep_override if desired loop rate of 1 is needed\n')
        print(f'::: RollPitchYaw: \t{rollpitchyaw_array}')
        print(f'::: Amplitude: \t\t{amplitude_array}')
        print(f'::: Period: \t\t{period_array}')
        print(f'::: Phase Shift: \t{phase_array}')
        print(f'::: Offset: \t\t{offset_array}')
        print(f'::: Repeat Loop: \t{loop_repeats} times\n')
        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')
        a = datetime.datetime.now()

    if dev_check:
        print("Check set values and hit 'enter' to execute. 'ctrl + c' to abort.")
        input()

    if printer:
        a = datetime.datetime.now()

    start = time.time()

    i = 0
    for timestep in range(0, publish_hz*loop_repeats):
        i += 1
        
        # Set highCmd values 
        hcmd.mode = MotorModeHigh.FORCE_STAND   
        hcmd.euler = np.multiply(np.array([
            amplitude_array[0] * math.sin(((math.pi)*2 / period_array[0]) * ((timestep / publish_hz) + phase_array[0])) + offset_array[0],
            amplitude_array[1] * math.sin(((math.pi)*2 / period_array[1]) * ((timestep / publish_hz) + phase_array[1])) + offset_array[1],
            amplitude_array[2] * math.sin(((math.pi)*2 / period_array[2]) * ((timestep / publish_hz) + phase_array[2])) + offset_array[2]]),
            rollpitchyaw_array)
        
        if timestep % 10 == 0 and printer:                  # Print every 10 cycles
            if timestep % publish_hz == 0 and printer:      # Print once per loop to check loop rate (should be 1.0 secs)
                b = datetime.datetime.now()     
                c = b - a
                a = datetime.datetime.now()
                print(f'TimeStep: {timestep:04d} \t  Euler RPY Angle: {hcmd.euler} \t LoopRate: {c.total_seconds()} secs\n')
                continue
            # print(f'TimeStep: {timestep:04d} \t  Euler RPY Angle: {hcmd.euler}\n')  # Comment out if you dont want to see every timestep

        cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
        conn.send(cmd_bytes)                    # Send the command


        remaining_delay = max(start + (i * sleep_rate) - time.time(), 0)
        # print("remaining delay: %s" % remaining_delay) # Uncomment to see remaining delay

        time.sleep(remaining_delay)        # Sleep for the remaining delay


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

### ToDo:
# Add an absolute value shifter [-1, 0, 1] but this might make the motion choppy at point of shift
# Add a function to change the period of the sin function
# Add a function to change the amplitude of the sin function
# Add a function to change the offset of the sin function
# Add a function to change the phase shift of the sin function
# Add a function to change the number of cycles of the sin function
# Create a class that can be used to store the above functions
# Create a cosine function
# Create a function that can be used to create a smooth transition between two sin functions
# Start doing motion such as walking in circle or line or spinning on the spot or jumping 
###
