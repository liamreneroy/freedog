from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType, SpeedLevel
from ucl.complex import motorCmd, led
import time
import math



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


def roll_sin(conn, hcmd, publish_hz=250, granularity_override=None, amplitude=0.7, frequency=1, reversed=False):
    '''ARG RANGES:
    publish_hz = [50, 500] (Hz) -> smaller equals faster execution time. If robot acting weird, try decreasing this value. Recommended 100+ 
    
    granularity_override = [100, 4000] (number of cycles) -> smaller equals slightly faster but choppier execution. Recommended 500+ for smooth motion.
    
    amplitude = [0, 0.8] (radians) -> smaller equals less roll angle. Recommended 0.6 for full roll angle.
    
    frequency = [1, 3] (cycles) -> quantity equals number of oscillations. Recommended 1 for one oscillation. 
                                   Value over 1 may trigger an override of publish_hz and granularity.
    
    reversed = [True, False] -> True reverses the direction of the roll_sin
    
    
    Note: You want to proportionally set your publishing_hz and granularity. 
          If publish_hz is high [250], granularity should likely be set to 2-3x that value [500-750]. 
    '''

    granularity_multiplier = 3   # Either 2x or 3x publish_hz is recommended

    # Check if frequency is valid
    if isinstance(frequency, int) == False or frequency > 3:
        raise ValueError("frequency must be integer and within range [1, 3]")

    if granularity_override:
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

    print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
    print('>> Executing: roll_sin ::: Publishing at {}Hz\n'.format(publish_hz))
    print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')

    for timestep in range(0, granularity+1):    # Not sure if I need the +1 but this makes it start/end at zero
        time.sleep(float(1./publish_hz))        # Average  --> Sleep to achieve desired 0.002 time interval (in seconds)

        # Set highCmd values 
        hcmd.mode = MotorModeHigh.FORCE_STAND   
        hcmd.euler = [amplitude * math.sin(frequency * timestep / granularity * ((math.pi)*2)), 0, 0]

        if timestep % 10 == 0:                  #Print every 10 cycles
            print(f"TimeStep: {timestep} ,  Roll Angle: {hcmd.euler}\n")

        cmd_bytes = hcmd.buildCmd(debug=False)  # Build the command
        conn.send(cmd_bytes)                    # Send the command


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



