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
import random

# random.seed(45)           # used for testing
random.seed(time.time())  # true randomness 

import modules.user_defined_functs as udf

import numpy as np
np.set_printoptions(formatter={'float': lambda x: '%7.4f' % (x)})
# np.set_printoptions(precision=3)



class MotorControl:

    '''
    Creates a MotorControl object. The arguments that need to be passed are:
        
        printer = ['all', 'minimal', None] -> depicts what will be printed when control is activated
    '''

    def __init__(self, time_signature=4, printer=None):
        '''
        You can use one of the 3 Presets WIFI_DEFAULTS, LOW_CMD_DEFAULTS or HIGH_CMD_DEFAULTS.
        IF NONE OF THEM ARE WORKING YOU CAN DEFINE A CUSTOM ONE LIKE THIS:

        MY_CONNECTION_SETTINGS = (listenPort, addr_wifi, sendPort_high, local_ip_wifi)
        conn = unitreeConnection(MY_CONNECTION_SETTINGS)
        '''

        # Set all self variables to empty values for initialization
        self.amplitude_array = None
        self.bpm = None
        self.bpm_limiter = 50
        self.control_function = None
        self.dev_check = None
        self.euler_array = None
        self.loop_repeats = None
        self.mode = None
        self.offset_array = None
        self.period_array = None
        self.phase_array = None
        self.printer = printer
        self.publish_hz = None
        self.sin_func_array = None
        self.sleep_override = None
        self.time_signature = time_signature
        self.timestep = None


        # Sets the printer argument
        self.print_options = ['all', 'minimal', None]   # Print options
        if isinstance(self.printer, str):
            self.printer = self.printer.lower()              

        # Check if printer is valid
        if self.printer not in self.print_options:          
            print(f"WARNING: printer [{self.printer}] must be one of the following: {self.print_options}\n::: Printer set to: None\n")
            self.printer = None


        self.conn = unitreeConnection(HIGH_WIFI_DEFAULTS)   # Creates a new connection object with HIGH_WIFI_DEFAULTS named 'conn'
        self.conn.startRecv()                               # Starts up connection
        self.hcmd = highCmd()                               # Creates the highCmd object named hcmd
        self.hstate = highState()                           # Creates the highState object named hstate


        cmd_bytes = self.hcmd.buildCmd(debug=False)         # Builds an empty command
        self.conn.send(cmd_bytes)                           # Send empty command to tell the dog the receive port and initialize the connection
        time.sleep(0.5)                                     # Some time to collect packets ;)

        print(">> MotorControl Initialized\n")        


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # GENERIC CONTROL FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def parse_data(self):
        ''' Parse data from the robot and print out the first packet'''
        
        if self.printer:
            print(">> Parsing Data\n")        

        data = self.conn.getData()              # Get the data from the connection object

        paket_count = 0                         # Parse the data
        for paket in data:                      
            self.hstate.parseData(paket)        

            if self.printer == 'all' and paket_count == 0:
                print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
                print(f'SN [{byte_print(self.hstate.SN)}]:\t{decode_sn(self.hstate.SN)}')
                print(f'Ver [{byte_print(self.hstate.version)}]:\t{decode_version(self.hstate.version)}')
                print(f'SOC:\t\t\t{self.hstate.bms.SOC} %')
                print(f'Overall Voltage:\t{getVoltage(self.hstate.bms.cell_vol)} mv') #something is still wrong here ?!
                print(f'Current:\t\t{self.hstate.bms.current} mA')
                print(f'Cycles:\t\t\t{self.hstate.bms.cycle}')
                print(f'Temps BQ:\t\t{self.hstate.bms.BQ_NTC[0]} °C, {self.hstate.bms.BQ_NTC[1]}°C')
                print(f'Temps MCU:\t\t{self.hstate.bms.MCU_NTC[0]} °C, {self.hstate.bms.MCU_NTC[1]}°C')
                print(f'FootForce:\t\t{self.hstate.footForce}')
                print(f'FootForceEst:\t\t{self.hstate.footForceEst}')
                print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
                print()
                print()
            paket_count += 1

        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def recover_control(self):
        ''' Recover control and put robot into standing pose. Used at startup'''

        print(">> Executing: RECOVERY\n")           # Allows robot to start from any state: IDLE / FORCE_STAND / DAMPED
        self.hcmd.mode = MotorModeHigh.RECOVERY
        cmd_bytes = self.hcmd.buildCmd(debug=False) # Build the command
        self.conn.send(cmd_bytes)                   # Send the command
        time.sleep(1)                               # Sleep for 1 second

        print(">> Executing: FORCE_STAND\n")        # Put robot in into standing state to start
        self.hcmd.mode = MotorModeHigh.FORCE_STAND
        self.hcmd.euler = [0, 0, 0]
        self.hcmd.bodyHeight = 0.0
        cmd_bytes = self.hcmd.buildCmd(debug=False) # Build the command
        self.conn.send(cmd_bytes)                   # Send the command
        time.sleep(1)                               # Sleep for 1 second

        return
    
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def terminate_control(self):
        ''' Terminate control and break connection. Used at shutdown'''

        print(">> Executing: FORCE_STAND\n")            
        self.hcmd.mode = MotorModeHigh.FORCE_STAND      # Put robot back into neutral standing state
        self.hcmd.euler = [0, 0, 0]
        self.hcmd.bodyHeight = 0.0
        cmd_bytes = self.hcmd.buildCmd(debug=False)     # Build the stand command
        self.conn.send(cmd_bytes)                       # Send the stand command
        time.sleep(1)                                   # Sleep for 1 second

        print(">> Executing: STAND_DOWN\n")             
        self.hcmd.mode = MotorModeHigh.STAND_DOWN       # Puts robot into lay-down position
        cmd_bytes = self.hcmd.buildCmd(debug=False)     # Build the lay-down command
        self.conn.send(cmd_bytes)                       # Send the lay-down command
        time.sleep(1)                                   # Sleep for 1 second

        print(">> Executing: IDLE\n")                   
        self.hcmd.mode = MotorModeHigh.IDLE             # Puts robot into idle state
        cmd_bytes = self.hcmd.buildCmd(debug=False)     # Build the idle command
        self.conn.send(cmd_bytes)                       # Send the idle command
        time.sleep(0.5)                                 # Sleep for 0.5 second

        print(">> Executing: DAMPING\n")                
        self.hcmd.mode = MotorModeHigh.DAMPING          # Puts robot into damped state
        cmd_bytes = self.hcmd.buildCmd(debug=False)     # Build the final command
        self.conn.send(cmd_bytes)                       # Send the final command
        time.sleep(1)                                   # Sleep for 1 second
        
        return True                                     # Return True to break connection

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def override_sleep_rate(self):
        ''' Override sleep rate based on current self.sleep_override '''
        
        print(f"WARNING: Overriding sleep_rate to {self.sleep_override}")
        print(f"::: BPM of {self.bpm} will be broken]")

        self.sleep_rate = self.sleep_override
        self.calculate_loop_rate()
        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def calculate_loop_rate(self):
        ''' Calculate loop_rate. Should be 1 if sleep_override is not used.'''

        self.loop_rate = self.sleep_rate * self.publish_hz   # Loop rate = number of published mgs x seconds to sleep between msgs 
        if self.printer == 'all':
            print(f"::: Resulting loop_rate: \t{self.loop_rate:0.3f} (Approx. {self.loop_rate:0.3f} secs per loop)\n")
        
        return
    
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def change_bpm(self, new_bpm):

        '''
        Changes the bpm and sets the sleep_rate accordingly.
        '''

        # Check if new bpm is valid 
        if new_bpm <= 0.0:
            print(f"WARNING: Attempt to change BPM to negative value {new_bpm} is not allowed")
            if self.bpm:
                print(f"::: BPM remains at: {self.bpm}\n")
                return
            
            elif self.bpm == None:
                self.bpm = 30.

        else:
            self.bpm = new_bpm
        
        while self.bpm > self.bpm_limiter:       # Sets motion limiter
            self.bpm /= 2. 

        bpm_to_bps_factored = (60. / self.bpm)    
        self.sleep_rate = (1. / int(self.publish_hz)) * bpm_to_bps_factored 
        
        if self.printer == 'all':
            print(f">> BPM set to: {self.bpm}")
            print(f"::: Resulting sleep rate: \t{self.sleep_rate:0.4f} (Sleeps {self.sleep_rate:0.4f} secs per msg)")

        self.calculate_loop_rate()

        return
    
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def value_check(self):

        ### GENERIC CHECKS ### 

        # Check if mode valid
        if self.control_function == 'sin_euler_ctrl':
            if self.mode not in self.sin_euler_mode_list:
                print(f"WARNING: mode {self.mode} is not in mode list: {self.sin_euler_mode_list}")
                self.mode = 'default'
                print(f"::: Mode set to: {self.mode}\n")
        
        elif self.control_function == 'push_up_ctrl':
            pass

        elif self.control_function == 'pose_ctrl':
            pass


        # Check if publish_hz valid
        if isinstance(self.publish_hz, int) == False or self.publish_hz > 1000 or self.publish_hz < 50:
            raise ValueError("publish_hz must be integer and within range [50, 1000]\n")
        

        # If sleep_override exists, check if its valid
        if self.sleep_override:
            if self.sleep_override > 0.1 or self.sleep_override < 0.001:
                raise ValueError("sleep_override must be within range [0.001, 0.1]\n")


        # Check if loop_repeats valid
        if self.loop_repeats > 128 or self.loop_repeats < 1:
            raise ValueError("loop_repeats must be within range [1, 128]\n")


        # Check if sleep_rate valid
        if self.sleep_rate < 0.002:
            print(f"WARNING: low sleep_rate of {self.sleep_rate} is not recommended. Might experience control lag") 
            print(f"::: Recommended sleep_rate >= 0.002. Decrease publish_hz to 500 or less or override sleep_rate\n")
            if self.sleep_rate < 0.001:
                raise ValueError("sleep_rate must be within range [0.001, 0.1]\n")


        # Check if loop_rate valid
        if self.loop_rate < 1.0:
            print(f'WARNING: Loop rate of {self.loop_rate} is less than 1 [1sec/loop]')
            print('::: Robot may move quickly and could damage itself\n')


        ### FUNCTION-SPECIFIC CHECKS ### 
        
        # sin_euler_ctrl only:
        if self.control_function == 'sin_euler_ctrl':
            
            # Check sin functions 
            for sin_function in range(len(self.sin_func_array)):
                if self.sin_func_array[sin_function] not in self.oscillating_functions:
                    raise ValueError(f"sin_function at idx [{sin_function}] must be one of the following: {self.oscillating_functions}\n")

            # Check period_array
            for period in self.period_array:
                if isinstance(period, float) == False or period > 4. or period < 1.:
                    print(f'WARNING: period values should be specified within range [1., 4.] for continuous motion')
                    if isinstance(period, float) == False or period > 16. or period < 1.:
                        raise ValueError("period values must be float within range [1., 16.] for each euler angle\n")

            # Check if loop_repeats is a multiple of max period
            if self.loop_repeats < max(self.period_array) or self.loop_repeats % max(self.period_array) != 0:
                print(f"WARNING: loop_repeats {self.loop_repeats} is not a multiple of max period {max(self.period_array)}")
                print(f"::: Robot may not complete full oscillation\n")

            # Check if euler is valid
            for euler in self.euler_array:
                if euler != 0 and euler != 1:
                    raise ValueError("euler values must be 0 or 1\n")

            # Check if amplitude + offset is valid
            for index_012 in range(0,3):
                if abs(self.amplitude_array[index_012]) + abs(self.offset_array[index_012]) > 0.55:
                    raise ValueError("sum of (amplitude + offset) must be within range [0, 0.55] for each euler angle\n")

            # Check if phase is within range [-2pi, 2pi]
            phase_counter = 0
            for phase in self.phase_array:
                if phase >= 2.:
                    while phase >= 2.:
                        phase -= 2.    
                    print(f'WARNING: phase values should be specified within range [-2., 2.]')
                    print(f'::: Overriding phase[{phase_counter}] of {self.phase_array[phase_counter]} to: {phase}\n')
                    self.phase_array[phase_counter] = phase
                    phase_counter += 1

                elif phase <= -2.:
                    while phase <= -2.:
                        phase += 2.    
                    print(f'WARNING: phase values should be specified within range [-2., 2.]')
                    print(f'::: Overriding phase[{phase_counter}] of {self.phase_array[phase_counter]} to: {phase}\n')
                    self.phase_array[phase_counter] = phase
                    phase_counter += 1


        if self.control_function == 'push_up_ctrl':
            pass

        if self.control_function == 'pose_ctrl':
            pass
        
        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def status_printer(self):
        if self.control_function == 'sin_euler_ctrl':
            print('\n+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
            print(f'>> Executing: [{self.control_function}] in [{self.mode}] mode @ timestep: {self.timestep:05d}\n')
            print(f'::: Publish Rate: {self.publish_hz} hz \t({self.publish_hz} messages per loop)')
            print(f'::: Beats Per Min: {self.bpm} bpm \t({self.bpm} loops per minute)')
            print(f'::: Sleep Rate: {self.sleep_rate:0.4f} secs \t(Sleeps {self.sleep_rate:0.4f} secs per msg)\n')
            print(f'RESULTING LOOP RATE: {self.loop_rate:0.4f} \t(Approx. {self.loop_rate:0.4f} secs per loop)\n')
            print(f'::: Euler Active: \t{self.euler_array}')
            print(f'::: Control Funcs:\t[{self.sin_func_array[0].__name__}  {self.sin_func_array[1].__name__}  {self.sin_func_array[2].__name__}]')
            print(f'::: Amplitude: \t\t{self.amplitude_array}')
            print(f'::: Period: \t\t{self.period_array}')
            print(f'::: Phase Shift: \t{self.phase_array}')
            print(f'::: Offset: \t\t{self.offset_array}')
            print(f'::: Repeat Loop: \t{self.loop_repeats} times\n')
            print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')
        
        elif self.control_function == 'push_up_ctrl':
            print('\n+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
            print(f'>> Executing: [{self.control_function}] in [{self.mode}] mode @ timestep: {self.timestep:05d}\n')
            print(f'::: Publish Rate: {self.publish_hz} hz \t({self.publish_hz} messages per loop)')
            print(f'::: Beats Per Min: {self.bpm} bpm \t({self.bpm} loops per minute)')
            print(f'::: Sleep Rate: {self.sleep_rate:0.4f} secs \t(Sleeps {self.sleep_rate:0.4f} secs per msg)\n')
            print(f'RESULTING LOOP RATE: {self.loop_rate:0.4f} \t(Approx. {self.loop_rate:0.4f} secs per loop)\n')
            print(f'::: Repeat Loop: \t{self.loop_repeats} times\n')
            print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')

        elif self.control_function == 'pose_ctrl':
            print('\n+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
            print(f'>> Executing: [{self.control_function}] in [{self.mode}] mode @ timestep: {self.timestep:05d}\n')
            print(f'::: Publish Rate: {self.publish_hz} hz \t({self.publish_hz} messages per loop)')
            print(f'::: Beats Per Min: {self.bpm} bpm \t({self.bpm} loops per minute)')
            print(f'::: Sleep Rate: {self.sleep_rate:0.4f} secs \t(Sleeps {self.sleep_rate:0.4f} secs per msg)\n')
            print(f'RESULTING LOOP RATE: {self.loop_rate:0.4f} \t(Approx. {self.loop_rate:0.4f} secs per loop)\n')
            print(f'::: Repeat Loop: \t{self.loop_repeats} times\n')
            print(f'>> Pose Parameters:\nROLL: {self.roll}\nPITCH: {self.pitch}\nYAW: {self.yaw}\nBODY HEIGHT: {self.body_height}\nBODY ORIENTATION: {self.body_orientation}\nPOSE DURATION: {self.pose_duration}\nVELOCITY: {self.velocity}\nSMOOTHNESS: {self.smoothness}\n')
            print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')


        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # SIN EULER CONTROL FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def sin_euler_ctrl(self, mode='default', publish_hz=200, bpm=30,
                                force_bpm_limiter=None,
                                sleep_override=None, loop_repeats=8, 
                                euler_array=np.array([1, 1, 1]),
                                sin_func_array=np.array([math.sin, math.sin, math.sin]),
                                amplitude_array=np.array([0.5, 0.5, 0.4]),
                                offset_array=np.array([0.0, 0.0, 0.0]), 
                                period_array=np.array([2.0, 2.0, 2.0]), 
                                phase_array=np.array([0.0, 0.0, 0.0]), 
                                dev_check=True):
        '''ARG RANGES:
        recall y = Asin(B(x + C)) + D   ///   A = amplitude, 2pi/B = period, C = phase shift, D = offset

        mode = ['default', 'rand_dance', '...'] ->  default = robot will run specified euler angles on repeat
                                                    rand_dance = robot will do random dance with random euler angles

        publish_hz = [50, 1000] (Hz - int) ->   Number of msgs being sent per loop. Smoother motion at higher hz but more comp expensive. 
                                                If robot acting undesirably, try decreasing this value. Recommended ~200+ 
        
        bpm = [1, 60] (bpm - int) ->    Sets the bpm at which the loops of the robot execute. Default = 1 loop/sec = 1 beat per sec = 60BPM. 
                                        Any specified BPM greater than 60 will be halved until within range [0-60]. Recommended value: 30-60                                             

        force_bpm_limiter = [int, None] ->  Forces BPM limiter ~ sets max BPM range [0, bpm_limiter]. 
                                             Robot cant move faster than 60/60=1.0 sec/loop (1.00 loop/sec) 

        sleep_override = [0.001, 0.1] (int) ->  Number of seconds each loop will sleep for. Setting this breaks the loop_rate=1 [1sec/loop]
                                                Smaller equals faster loops, but more comp expensive. If you have high publish_hz [200+], then you 
                                                can use this to slow down the loops to get smoother motion.
        
        loop_repeats = [1, 128] (int)-> Number of times to repeat the loop. Recommended to choose numbers that are 2^n.

        euler_array = [0, 1] -> 1/True enables the sin_roll, sin_pitch, sin_yaw respectively. 0/False disables the sin_roll, sin_pitch, sin_yaw respectively.

        sin_func_array = [math.sin, udf.neg_sin, ... ] -> Continuous functions from math or user_defined 'udf' library.

        amplitude_array = [-0.55, 0.55] (radians - float) -> Larger value equals bigger angle. Negative is reversed. Recommended 0.6 for full angle.
                                                             CAREFUL WITH THIS ONE.. dont max joint limits. Set skip_check=False first to check your code.
        
        offset_array = [-0.55, 0.55] (radians - float) -> Offsets the oscillation. Combination of offset and amplitude cannot exceed 0.7.
                                                          CAREFUL WITH THIS ONE.. dont max joint limits. Set skip_check=False first to check your code.

        period_array = [1, 16] (cycles - float) ->  Period of oscillation. Recommended 1 for one oscillation. 2 gives oscillation which takes twice as long.
        
        phase_array = [-2, 2] (radians - float) ->  Phase shift of oscillation in radians. 1 for half phase shift. 2=2pi and -2=-2pi
                                                    For sin(), phase recommended to be 0. 
        
        dev_check = [True, None] -> If True, will check if all values are within range. Also asks for input before execution.
                                    If False, will skip all checks.

        Note:   loop_rate of 1 = [1sec/loop] -> useful for making robot move to a given time signature..
                The following equation is used to calculate loop_rate: 
                
                loop_rate = sleep_rate * publish_hz
                sleep_rate = 1 / publish_hz

                This loop_rate will keep to 1 so long as sleep_override is not used.
                You can increase publish_hz and override the sleep_rate to get smoother motion while breaking loop rate.  
        '''

        # Set the control function
        self.control_function = 'sin_euler_ctrl'

        # Reset BPM limiter if forced
        if force_bpm_limiter:
            self.bpm_limiter = force_bpm_limiter

        # Set all values to self.values
        self.amplitude_array = amplitude_array
        self.dev_check = dev_check
        self.euler_array = euler_array
        self.loop_repeats = loop_repeats
        self.mode = mode
        self.offset_array = offset_array
        self.oscillating_functions = [math.sin, udf.neg_sin, udf.abs_sin, udf.neg_abs_sin]
        self.period_array = period_array
        self.phase_array = phase_array
        self.publish_hz = publish_hz
        self.sin_euler_mode_list = ['default', 'rand_dance']
        self.sin_func_array = sin_func_array
        self.sleep_override = sleep_override
        self.timestep = 0

        # Changes bpm and sets sleep_rate
        self.change_bpm(bpm)   


        # Override sleep_rate if sleep_override is set
        if self.sleep_override:
            self.override_sleep_rate()

        # Check if all values are within range // skip ONLY if you are very confident in your code >:) 
        if self.dev_check:
            self.value_check()

        if self.dev_check or self.printer == 'all':
            self.status_printer()

        if self.dev_check:
            print("Check set values and hit 'enter' to execute. 'ctrl + c' to abort.")
            input()

        if self.printer:
            a = datetime.datetime.now()

        start = time.time()

        for self.timestep in range(0, self.publish_hz*self.loop_repeats):
            
            # Set highCmd values 
            self.hcmd.mode = MotorModeHigh.FORCE_STAND   
            self.hcmd.euler = np.multiply(np.array([

                # Y = euler angle = Asin(B(x + C)) + D
                # X = self.timestep / self.publish_hz
                # A = self.amplitude_array[0]
                # B = 2*math.pi / self.period_array[0]
                # C = self.phase_array[0]
                # D = self.offset_array[0]

                self.amplitude_array[0] * self.sin_func_array[0](2*math.pi / self.period_array[0] * ((self.timestep / self.publish_hz) + self.phase_array[0])) + self.offset_array[0],
                self.amplitude_array[1] * self.sin_func_array[1](2*math.pi / self.period_array[1] * ((self.timestep / self.publish_hz) + self.phase_array[1])) + self.offset_array[1],
                self.amplitude_array[2] * self.sin_func_array[2](2*math.pi / self.period_array[2] * ((self.timestep / self.publish_hz) + self.phase_array[2])) + self.offset_array[2]]),
                self.euler_array)

            if self.timestep % self.publish_hz == 0 and self.printer:        # Print once per loop to check loop rate (should be 1.0 secs)
                b = datetime.datetime.now()     
                c = b - a
                a = datetime.datetime.now()
                print(f'TimeStep: {self.timestep:05d} \tEuler RPY Angle: {self.hcmd.euler} \t(Real LR: {c.total_seconds():0.4f} // Targ LR: {self.loop_rate:0.4f} secs)\n')
                
            elif self.timestep % 10 == 0 and self.printer:                                      # Print every 10th timestep
                print(f'TimeStep: {self.timestep:05d} \tEuler RPY Angle: {self.hcmd.euler}\n')  # Comment out if you dont want to see every 10th timestep

            cmd_bytes = self.hcmd.buildCmd(debug=False)  # Build the command
            self.conn.send(cmd_bytes)                    # Send the command


            # Random Dance Logic
            if self.mode == 'rand_dance':
                self.sin_euler_dance_logic()

            remaining_delay = max(start + ((self.timestep+1) * self.sleep_rate) - time.time(), 0) # Calculate delay
            # print("remaining delay: %s" % remaining_delay)                                      # Uncomment to see remaining delay

            time.sleep(remaining_delay)        # Sleep for the remaining delay

        
        print(f'>> End of: sin_euler_ctrl in [{self.mode}] mode\n')        
        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')

        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def sin_euler_rand_params(self):
        ''' Randomize euler_array and amplitude_array '''
        
        self.euler_array = np.array([1, 1, 1])

        self.amplitude_array = np.array([random.uniform(0.45, 0.5) * random.choice([-1, 1]), 
                                         random.uniform(0.5, 0.55) * random.choice([-1, 1]),
                                         random.uniform(0.35, 0.4) * random.choice([-1, 1])])
        
        period_options = [1., 2., 4.]
        self.period_array = np.array([1., 1., 1.])
        
        while np.sum(self.period_array) <= 3:
            self.period_array = np.array([random.choice(period_options), random.choice(period_options), random.choice(period_options) ])

        # Set phase_array (this shouldn't change if you're only using sin functions)
        self.phase_array=np.array([0., 0., 0.]) 

        if self.printer:
            print(f">> New Traject @ timestep:\t{self.timestep:05d}")
            print(f">> New Euler Amplitude:\t\t[R, P, Y] = {self.amplitude_array}")
            print(f">> New Euler Period:   \t\t[R, P, Y] = {self.period_array}")
        
        return
    
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def sin_euler_rand_sin(self):
        ''' Randomly selects one of the variations of sin() functions found in self.oscillating_functions.'''

        # Always keen one as normal sin() function
        self.sin_func_array = [math.sin, 
                               math.sin, 
                               random.choice(self.oscillating_functions)]      
        
        # Shuffle order
        random.shuffle(self.sin_func_array)

        if self.printer:
            print(f'>> New Euler Ctrl Func:\t\t[R, P, Y] = [{self.sin_func_array[0].__name__}  {self.sin_func_array[1].__name__}  {self.sin_func_array[2].__name__}]\n')
        
        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def sin_euler_dance_logic(self):
        ''' After every 2nd loop, do one of three things:
            A: Change ctrl and euler angles
            B: Change ctrl and euler angles and reverse trajectory
            C: Reverse trajectory
            '''

        if self.timestep % (self.publish_hz * 2) == 0:
            next_move = random.randint(0, 3)        

            if next_move == 0 or next_move == 1:
                self.sin_euler_rand_params()        # New euler angles and params
                self.sin_euler_rand_sin()      # New ctrl functions
                self.value_check()        # Checks values for safety
                if self.printer == 'all':
                    self.status_printer()       

            elif next_move == 2:
                self.sin_euler_rand_params()        # New euler angles and params
                self.sin_euler_rand_sin()      # New ctrl functions
                self.value_check()        # Checks values for safety
                if self.printer == 'all':
                    self.status_printer()       
                if self.printer:
                    print(f'>> Reverse Traject @ timestep: \t{self.timestep:05d}\n')
                self.amplitude_array = self.amplitude_array * -1

            elif next_move == 3:
                if self.printer:
                    print(f'>> Reverse Traject @ timestep: \t{self.timestep:05d}\n')
                self.amplitude_array = self.amplitude_array * -1

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # PUSH UP CONTROL FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def push_up_ctrl(self, mode='default', publish_hz=200, bpm=30, 
                            force_bpm_limiter=None,
                            sleep_override=None, loop_repeats=5,  
                            dev_check=True):
        
        '''ARG RANGES:
        mode = ['default'] -> specific mode robot will follow when running this function

        publish_hz = [50, 1000] (Hz - int) ->   Number of msgs being sent per loop. Smoother motion at higher hz but more comp expensive. 
                                                If robot acting undesirably, try decreasing this value. Recommended ~200+ 
        
        bpm = [1, 60] (bpm - int) ->    Sets the bpm at which the loops of the robot execute. If nothing is set, the robot will move
                                        at a rate of one control loop per second -> 0.5 loop/sec = 1 loop per 2 secs = 30BPM. 
                                        Any specified BPM greater than 40 will be halved until within range [0-40]. Recommended value: 30                                             

        force_bpm_limiter = [int, None] ->  Forces BPM limiter ~ sets max BPM range [0, bpm_limiter]. 
                                            Robot cant move faster than 60/60=1.0 sec/loop (1.00 loop/sec) 
                                        
        sleep_override = [0.001, 0.1] (int) ->  Sets the number of seconds that each loop will sleep for. Setting this breaks the loop_rate=1 [1sec/loop]
                                                Smaller equals faster loops, but more comp expensive. If you have high publish_hz [200+], then you 
                                                can use this to slow down the loops to get smoother motion.
        
        loop_repeats = [1, 128] (int)-> Number of times to repeat the loop. 1 loop = 1 push up. 

        dev_check = [True, None] -> If True, will check if all values are within range. Also asks for input before execution.
                                    If False, will skip all checks.
                
        Note:   The following equation is used to calculate loop_rate: 
                
                loop_rate = sleep_rate * publish_hz
                sleep_rate = 1 / publish_hz

                This loop_rate will keep to 1 so long as sleep_override is not used.
                You can increase publish_hz and override the sleep_rate to get smoother motion while breaking loop rate.  
        '''

        # Calculate sleep_rate while taking into account bpm_override if it exists

        # Set the control function
        self.control_function = 'push_up_ctrl'

        # Reset BPM limiter if forced
        if force_bpm_limiter:
            self.bpm_limiter = force_bpm_limiter

        # Set all values to self.values
        self.dev_check = dev_check
        self.loop_repeats = loop_repeats
        self.mode = mode
        self.push_up_mode_list = ['default'] 
        self.publish_hz = publish_hz
        self.sleep_override = sleep_override
        self.timestep = 0

        # Changes bpm and sets sleep_rate
        self.change_bpm(bpm)

        # Override sleep_rate if sleep_override is set
        if self.sleep_override:
            self.override_sleep_rate()

        # Check if all values are within range // skip ONLY if you are very confident in your code >:) 
        if self.dev_check:
            self.value_check()

        if self.dev_check or self.printer == 'all':
            self.status_printer()

        if self.dev_check:
            print("Check set values and hit 'enter' to execute. 'ctrl + c' to abort.")
            input()

        if self.printer:
            a = datetime.datetime.now()

        start = time.time()

        for self.timestep in range(0, self.publish_hz*self.loop_repeats):
            
            # Set highCmd values 
            if self.timestep % self.publish_hz == 0:
                self.hcmd.mode = MotorModeHigh.STAND_DOWN     # Put robot back into neutral standing state
                self.hcmd.euler = [0, 0, 0]

            elif self.timestep % self.publish_hz == self.publish_hz/2:
                self.hcmd.mode = MotorModeHigh.STAND_UP      # Put robot back into neutral standing state
                self.hcmd.euler = [0, 0, 0]
                self.hcmd.bodyHeight = 0.0

            # Loop rate and print statements ~ print once per loop to check loop rate (should be 1.0 secs)
            if self.timestep % self.publish_hz == 0 and self.printer:        
                b = datetime.datetime.now()     
                c = b - a
                a = datetime.datetime.now()
                print(f'TimeStep: {self.timestep:05d} \tDown: {int(self.timestep / self.publish_hz):03d}\t(Real LR: {c.total_seconds():0.4f} // Targ LR: {self.loop_rate:0.4f} secs)\n')
            
            elif self.timestep % self.publish_hz == self.publish_hz/2 and self.printer:
                print(f'TimeStep: {self.timestep:05d} \tUp:   {int((self.timestep-self.publish_hz/2) / self.publish_hz):03d}\n')
            
            elif self.timestep % 10 == 0 and self.printer:      # Print every 10th timestep
                print(f'TimeStep: {self.timestep:05d} \n')      # Comment out if you dont want to see every 10th timestep

            cmd_bytes = self.hcmd.buildCmd(debug=False)         # Build the command
            self.conn.send(cmd_bytes)                           # Send the command

            remaining_delay = max(start + ((self.timestep+1) * self.sleep_rate) - time.time(), 0) 
            # print("remaining delay: %s" % remaining_delay)                                      # Uncomment to see remaining delay

            time.sleep(remaining_delay)        # Sleep for the remaining delay

        print('final', self.hcmd.mode)
        print(f'>> End of: push_up_ctrl\n')        
        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')

        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # POSE CONTROL FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def pose_ctrl(self, mode='default', publish_hz=200, 
                            bpm=60, bars=4, loop_repeats=1,
                            force_bpm_limiter=60,
                            use_param_time=True,
                            delay_start=0.0,  
                            sleep_override=None,
                            move_to_pose_base_time = 1.0,
                            pose_raw_param_dict = {'roll': 'neutral', 
                                                    'pitch': 'neutral', 
                                                    'yaw': 'neutral', 
                                                    'body_height': 'normal', 
                                                    'body_orientation': 'user',
                                                    'pose_duration': 'medium',
                                                    'velocity': 'normal',
                                                    'smoothness': 'smooth'},
                            dev_check=True):


        # Set the control function
        self.control_function = 'pose_ctrl'

        # Reset BPM limiter if forced
        if force_bpm_limiter:
            self.bpm_limiter = force_bpm_limiter

        # Set all values to self.values
        self.dev_check = dev_check
        self.loop_repeats = loop_repeats
        self.mode = mode
        self.push_up_mode_list = ['default'] 
        self.publish_hz = publish_hz
        self.sleep_override = sleep_override
        self.timestep = 0

        # Changes bpm and sets sleep_rate
        self.change_bpm(bpm)

        # Override sleep_rate if sleep_override is set
        if self.sleep_override:
            self.override_sleep_rate()


        # Convert motion params to numeric equivalents 
        pose_conv_param_dict = self.pose_param_conversion(pose_raw_param_dict)


        # Set all converted values of motion params to self.values
        self.roll = pose_conv_param_dict['roll']
        self.pitch = pose_conv_param_dict['pitch']
        self.yaw = pose_conv_param_dict['yaw']
        self.body_height =  pose_conv_param_dict['body_height']
        self.body_orientation = pose_conv_param_dict['body_orientation']
        self.pose_duration = pose_conv_param_dict['pose_duration']
        self.velocity = pose_conv_param_dict['velocity']
        self.smoothness = pose_conv_param_dict['smoothness']

        # Special condition for pose duration
        if use_param_time == False:
            self.pose_duration = (self.bpm/60) * self.time_signature * bars  # Confirm this later


        # Check if all values are within range // skip ONLY if you are very confident in your code >:) 
        if self.dev_check:
            self.value_check()
        
        if self.dev_check or self.printer == 'all':
            self.status_printer()

        if self.dev_check:
            print("Check set values and hit 'enter' to execute. 'ctrl + c' to abort.")
            input()


        # If there is a time delay, sleep first
        time.sleep(delay_start)


        for i in range(0, self.loop_repeats):
            if self.printer:
                a = datetime.datetime.now()

            # Start the loop timer
            start = time.time()
            
            move_step_qty = int((self.publish_hz*move_to_pose_base_time)/self.velocity)
            array_noise = np.random.uniform(-self.smoothness, self.smoothness, move_step_qty)

            # Task A: Move to pose
            print(">> Task A: Move to Pose\n")

            roll_steps_a_array = np.linspace(0, self.roll, move_step_qty)
            pitch_steps_a_array = np.linspace(0, self.pitch, move_step_qty)
            yaw_steps_a_array = np.linspace(0, self.yaw, move_step_qty)
            body_height_steps_a_array = np.linspace(0, self.body_height, move_step_qty)

            # Add the noise if smoothness is set:
            if pose_raw_param_dict['smoothness'] == 'shaky':
                roll_steps_a_array[int(math.ceil(self.publish_hz/5)):] = [x + y for x, y in zip(roll_steps_a_array[int(math.ceil(self.publish_hz/5)):], array_noise[int(math.ceil(self.publish_hz/5)):])]
                pitch_steps_a_array[int(math.ceil(self.publish_hz/5)):] = [x + y for x, y in zip(pitch_steps_a_array[int(math.ceil(self.publish_hz/5)):], array_noise[int(math.ceil(self.publish_hz/5)):])]
                yaw_steps_a_array[int(math.ceil(self.publish_hz/5)):] = [x + y for x, y in zip(yaw_steps_a_array[int(math.ceil(self.publish_hz/5)):], array_noise[int(math.ceil(self.publish_hz/5)):])]
                body_height_steps_a_array[int(math.ceil(self.publish_hz/5)):] = [x + y for x, y in zip(body_height_steps_a_array[int(math.ceil(self.publish_hz/5)):], array_noise[int(math.ceil(self.publish_hz/5)):])]

            for self.timestep in range (0, move_step_qty):

                # Set highCmd values 
                self.hcmd.mode = MotorModeHigh.FORCE_STAND   
                self.hcmd.euler = np.array([roll_steps_a_array[self.timestep], 
                                            pitch_steps_a_array[self.timestep],
                                            yaw_steps_a_array[self.timestep]])
                
                self.hcmd.bodyHeight = body_height_steps_a_array[self.timestep]

                cmd_bytes = self.hcmd.buildCmd(debug=False)  # Build the command
                self.conn.send(cmd_bytes)                    # Send the command

                if self.timestep % self.publish_hz == 0 and self.printer:        # Print once per loop to check loop rate (should be 1.0 secs)
                    b = datetime.datetime.now()     
                    c = b - a
                    a = datetime.datetime.now()
                    print(f'TimeStep: {self.timestep:05d} \tEuler RPY Angle: {self.hcmd.euler} \t Body Height: {self.hcmd.bodyHeight:6.3f}\t(Real LR: {c.total_seconds():0.4f} // Targ LR: {self.loop_rate:0.4f} secs)')
                    
                elif self.timestep % 10 == 0 and self.printer:                                      # Print every 10th timestep
                    print(f'TimeStep: {self.timestep:05d} \tEuler RPY Angle: {self.hcmd.euler} \t Body Height: {self.hcmd.bodyHeight:6.3f}')

                remaining_delay = max(start + ((self.timestep+1) * self.sleep_rate) - time.time(), 0)   # Calculate delay
                # print("remaining delay: %s" % remaining_delay)                                        # Uncomment to see remaining delay

                time.sleep(remaining_delay)        # Sleep for the remaining delay
            

            # Task B: Hold pose
            print("\n>> Task B: Hold pose\n")
            steps_b_range = int(move_step_qty + self.publish_hz*self.pose_duration) 

            for self.timestep in range (move_step_qty, steps_b_range):
            
                # Set highCmd values 
                self.hcmd.mode = MotorModeHigh.FORCE_STAND   
                self.hcmd.euler = np.array([self.roll + (np.random.uniform(-1, 1)*self.smoothness),
                                            self.pitch + (np.random.uniform(-1, 1)*self.smoothness),
                                            self.yaw + (np.random.uniform(-1, 1)*self.smoothness)])
                
                self.hcmd.bodyHeight = self.body_height + (np.random.uniform(-1, 1)*self.smoothness)


                cmd_bytes = self.hcmd.buildCmd(debug=False)  # Build the command
                self.conn.send(cmd_bytes)                    # Send the command


                if self.timestep % self.publish_hz == 0 and self.printer:        # Print once per loop to check loop rate (should be 1.0 secs)
                    b = datetime.datetime.now()     
                    c = b - a
                    a = datetime.datetime.now()
                    print(f'TimeStep: {self.timestep:05d} \tEuler RPY Angle: {self.hcmd.euler} \t Body Height: {self.hcmd.bodyHeight:6.3f}\t(Real LR: {c.total_seconds():0.4f} // Targ LR: {self.loop_rate:0.4f} secs)')
                    
                elif self.timestep % 10 == 0 and self.printer:                                      # Print every 10th timestep
                    print(f'TimeStep: {self.timestep:05d} \tEuler RPY Angle: {self.hcmd.euler} \t Body Height: {self.hcmd.bodyHeight:6.3f}')


                remaining_delay = max(start + ((self.timestep+1) * self.sleep_rate) - time.time(), 0) # Calculate delay
                # print("remaining delay: %s" % remaining_delay)                                      # Uncomment to see remaining delay

                time.sleep(remaining_delay)        # Sleep for the remaining delay
            

            # Task C: Return to neutral
            print("\n>> Task C: Move to Neutral\n")
            steps_c_range = int(steps_b_range + move_step_qty) 

            roll_steps_c_array = udf.reverse_array(roll_steps_a_array)
            pitch_steps_c_array = udf.reverse_array(pitch_steps_a_array) 
            yaw_steps_c_array = udf.reverse_array(yaw_steps_a_array)
            body_height_steps_c_array = udf.reverse_array(body_height_steps_a_array)

            for self.timestep in range (steps_b_range, steps_c_range):

                # Set highCmd values 
                self.hcmd.mode = MotorModeHigh.FORCE_STAND   
                self.hcmd.euler = np.array([roll_steps_c_array[self.timestep - steps_b_range], 
                                            pitch_steps_c_array[self.timestep - steps_b_range],
                                            yaw_steps_c_array[self.timestep - steps_b_range]])
                
                self.hcmd.bodyHeight = body_height_steps_c_array[self.timestep - steps_b_range]

                cmd_bytes = self.hcmd.buildCmd(debug=False)  # Build the command
                self.conn.send(cmd_bytes)                    # Send the command

                if self.timestep % self.publish_hz == 0 and self.printer:        # Print once per loop to check loop rate (should be 1.0 secs)
                    b = datetime.datetime.now()     
                    c = b - a
                    a = datetime.datetime.now()
                    print(f'TimeStep: {self.timestep:05d} \tEuler RPY Angle: {self.hcmd.euler} \t Body Height: {self.hcmd.bodyHeight:6.3f}\t(Real LR: {c.total_seconds():0.4f} // Targ LR: {self.loop_rate:0.4f} secs)')
                    
                elif self.timestep % 10 == 0 and self.printer:                                      # Print every 10th timestep
                    print(f'TimeStep: {self.timestep:05d} \tEuler RPY Angle: {self.hcmd.euler} \t Body Height: {self.hcmd.bodyHeight:6.3f}')

                remaining_delay = max(start + ((self.timestep+1) * self.sleep_rate) - time.time(), 0)   # Calculate delay
                # print("remaining delay: %s" % remaining_delay)                                        # Uncomment to see remaining delay

                time.sleep(remaining_delay)        # Sleep for the remaining delay

        print(f'\n>> End of: pose_ctrl in [{self.mode}] mode\n')        
        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')

        return

                
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def pose_param_conversion(self, param_dict):
        ''' Converts pose params to numeric equivalents '''
        conv_param_dict = {}

        # ROLL
        if param_dict['roll'] == 'left':
            conv_param_dict['roll'] = -0.5
        elif param_dict['roll'] == 'neutral':
            conv_param_dict['roll'] = 0.0
        elif param_dict['roll'] == 'right':
            conv_param_dict['roll'] = 0.5
        else:
            raise ValueError("roll must be one of the following: ['left', 'neutral', 'right']\n")


        # PITCH
        if param_dict['pitch'] == 'down':
            conv_param_dict['pitch'] = 0.5
        elif param_dict['pitch'] == 'neutral':
            conv_param_dict['pitch'] = 0.0
        elif param_dict['pitch'] == 'up':
            conv_param_dict['pitch'] = -0.5
        else:
            raise ValueError("pitch must be one of the following: ['down', 'neutral', 'up']\n")
        

        # YAW
        if param_dict['yaw'] == 'left':
            conv_param_dict['yaw'] = 0.3
        elif param_dict['yaw'] == 'neutral':
            conv_param_dict['yaw'] = 0.0
        elif param_dict['yaw'] == 'right':
            conv_param_dict['yaw'] = -0.3
        else:
            raise ValueError("yaw must be one of the following: ['left', 'neutral', 'right']\n")
        

        # BODY_HEIGHT
        if param_dict['body_height'] == 'low':
            conv_param_dict['body_height'] = -0.25
        elif param_dict['body_height'] == 'neutral':
            conv_param_dict['body_height'] = -0.05
        elif param_dict['body_height'] == 'high':
            conv_param_dict['body_height'] = 0.2
        else:
            raise ValueError("body_height must be one of the following: ['low', 'neutral', 'high']\n")
        

        # BODY_ORIENTATION (passive)
        conv_param_dict['body_orientation'] = param_dict['body_orientation']


        # POSE DURATION
        if param_dict['pose_duration'] == 'short':
            conv_param_dict['pose_duration'] = 1.0
        elif param_dict['pose_duration'] == 'medium':
            conv_param_dict['pose_duration'] = 4.0
        elif param_dict['pose_duration'] == 'long':
            conv_param_dict['pose_duration'] = 8.0
        else:
            raise ValueError("pose_duration must be one of the following: ['short', 'medium', 'long']\n")
        

        # VELOCITY
        if param_dict['velocity'] == 'slow':
            conv_param_dict['velocity'] = 0.5
        elif param_dict['velocity'] == 'normal':
            conv_param_dict['velocity'] = 1.0
        elif param_dict['velocity'] == 'fast':
            conv_param_dict['velocity'] = 2.0
        else:
            raise ValueError("velocity must be one of the following: ['slow', 'normal', 'fast']\n")
        

        # SMOOTHNESS
        if param_dict['smoothness'] == 'smooth':
            conv_param_dict['smoothness'] = 0
        elif param_dict['smoothness'] == 'shaky':
            conv_param_dict['smoothness'] = 0.1
        else:
            raise ValueError("smoothness must be one of the following: ['smooth', 'shaky']\n")

        if self.printer:
            print(f">> Raw Param Dict: {param_dict}\n")
            print(f">> Converted Param Dict: {conv_param_dict}\n")

        return conv_param_dict





# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ToDo List:
# Add a function to change any of the self values (BPM, amplitude, offset, period etc.) 
    # To do this you likely need the ability to have multiple functions running at once
    # This is where you need ROS


# Get the robot to walk in a line forward backwards 
# Get the robot to walk in a circle 
# Start doing motion such as walking in circle or line or spinning on the spot or jumping
