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
random.seed(15) # random.seed(time.time())  eventually use this

import user_defined_ctrl_functs as udcf

import numpy as np
np.set_printoptions(formatter={'float': lambda x: '%7.4f' % (x)})
# np.set_printoptions(precision=3)



class MotorControl:

    '''
    Creates a MotorControl object. The arguments that need to be passed are:
        
        printer = ['all', 'angles', None] -> depicts what will be printed when control is activated
    '''

    def __init__(self, printer=None):
        '''
        You can use one of the 3 Presets WIFI_DEFAULTS, LOW_CMD_DEFAULTS or HIGH_CMD_DEFAULTS.
        IF NONE OF THEM ARE WORKING YOU CAN DEFINE A CUSTOM ONE LIKE THIS:

        MY_CONNECTION_SETTINGS = (listenPort, addr_wifi, sendPort_high, local_ip_wifi)
        conn = unitreeConnection(MY_CONNECTION_SETTINGS)
        '''

        self.conn = unitreeConnection(HIGH_WIFI_DEFAULTS)   # Creates a new connection object with HIGH_WIFI_DEFAULTS named 'conn'
        self.conn.startRecv()                               # Starts up connection
        self.hcmd = highCmd()                               # Creates the highCmd object named hcmd
        self.hstate = highState()                           # Creates the highState object named hstate

        cmd_bytes = self.hcmd.buildCmd(debug=False)         # Builds an empty command
        self.conn.send(cmd_bytes)                           # Send empty command to tell the dog the receive port and initialize the connection
        time.sleep(0.5)                                     # Some time to collect packets ;)

        self.oscillating_functions = [math.sin, udcf.neg_sin, udcf.abs_sin, udcf.neg_abs_sin]  # removed math.cos
        self.mode_list = ['default', 'rand_dance']
        self.print_options = ['all', 'angles', None]

        self.printer = printer.lower()                              # Sets the printer

        # Check if printer is valid
        if self.printer not in self.print_options:
            print(f"WARNING: printer {self.printer} must be one of the following: {self.print_options}\n::: Printer set to: None\n")
            self.printer = None

        print(">> MotorControl Initialized\n")        

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
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def override_sleep_rate(self):
        ''' Override sleep rate based on current self.sleep_override '''
        
        print(f"WARNING: Overriding sleep_rate to {self.sleep_override}")
        print(f"::: BPM of {self.bpm} will be broken]")

        self.sleep_rate = self.sleep_override
        self.calculate_loop_rate()
        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def calculate_loop_rate(self):
        ''' Calculate loop_rate. Should be 1 if sleep_override is not used.'''

        self.loop_rate = self.sleep_rate * self.publish_hz   # Loop rate = number of published mgs x seconds to sleep between msgs 
        if self.printer == 'all':
            print(f"::: Resulting loop_rate: \t{self.loop_rate:0.3f} (Approx. {self.loop_rate:0.3f} secs per loop)\n")
        
        return
    
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def rand_euler(self):
        ''' Randomize euler_array and amplitude_array '''
        
        self.euler_array = np.array([1, 1, 1])  # could also use randint(-1,1) to get -1, 0, 1 and remove euler_array

        ## If you want to only activate 2/3 Euler angles at random, you can:
        #       self.euler_array = np.array([random.getrandbits(1), random.getrandbits(1), random.getrandbits(1)])
        #       while self.euler_array.sum() <= 1:  
        #           self.euler_array = np.array([random.getrandbits(1), random.getrandbits(1), random.getrandbits(1)])

        self.amplitude_array = np.array([random.uniform(0.4, 0.55) * random.choice([-1, 1]), 
                                         random.uniform(0.4, 0.55) * random.choice([-1, 1]),
                                         random.uniform(0.3, 0.4) * random.choice([-1, 1])])
        

        period_scheme = random.randint(0, 4)              # Set and randomize the period scheme     
        if period_scheme == 0:
            self.period_array = np.array([1., 1., 4.])
        elif period_scheme == 1:
            self.period_array = np.array([1., 2., 2.])
        elif period_scheme == 2:
            self.period_array = np.array([1., 2., 4.])
        elif period_scheme == 3:
            self.period_array = np.array([2., 2., 2.])  
        elif period_scheme == 4 and self.bpm <= 45:       # slightly different scheme if BPM<=45
            self.period_array = np.array([1., 1., 2.]) 
        elif period_scheme == 4 and self.bpm > 45:        # slightly different scheme if BPM>45
            self.period_array = np.array([2., 2., 4.]) 

        random.shuffle(self.period_array)

        # Set phase_array (this shouldn't change if you're only using sin functions)
        self.phase_array=np.array([0., 0., 0.]) 

        if self.printer:
            print(f">> New Euler Amplitude:\t\t[R, P, Y] = {self.amplitude_array}\n")
            print(f">> New Euler Period:   \t\t[R, P, Y] = {self.period_array}\n")
            print(f">> New Traject @ timestep:\t{self.timestep:05d}\n")
        
        return
    
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def rand_ctrl_func_array(self):
        ''' Randomly selects the function to be used. This method is currently not implemented.'''

        # Always keen one as normal sin() function
        self.ctrl_func_array = [math.sin, 
                                math.sin, 
                                random.choice(self.oscillating_functions)]      
        
        # Shuffle order
        random.shuffle(self.ctrl_func_array)

        if self.printer:
            print(f'>> New Euler Ctrl Func:\t\t[R, P, Y] = [{self.ctrl_func_array[0].__name__}  {self.ctrl_func_array[1].__name__}  {self.ctrl_func_array[2].__name__}] @ timestep: {self.timestep:05d}\n')
        
        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def change_bpm(self, new_bpm):

        # Check if new bpm is valid 
        if new_bpm <= 0.0:
            print(f"WARNING: Attempt to change BPM to negative value {new_bpm} is not allowed")
            if self.bpm:
                print(f"::: BPM remains at: {self.bpm}\n")
                return
            
            elif self.bpm == None:
                self.bpm = 60.

        else:
            self.bpm = new_bpm
        
        while self.bpm > 60.:  # Sets max BPM range [0, 60]. Robot cant move faster than 60/60=1.0 sec/loop (1.00 loop/sec)
            self.bpm /= 2. 

        bpm_to_bps_factored = (60. / self.bpm)              
        self.sleep_rate = (1. / int(self.publish_hz)) * bpm_to_bps_factored 
        
        if self.printer == 'all':
            print(f">> BPM set to: {self.bpm}")
            print(f"::: Resulting sleep rate: \t{self.sleep_rate:0.4f} (Sleeps {self.sleep_rate:0.4f} secs per msg)")

        self.calculate_loop_rate()

        return
    
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def status_printer(self):
        print('\n+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
        print(f'>> Executing: euler_control in [{self.mode}] mode @ timestep: {self.timestep:05d}\n')
        print(f'::: Publish Rate: {self.publish_hz} hz \t({self.publish_hz} messages per loop)')
        print(f'::: Beats Per Min: {self.bpm} bpm \t({self.bpm} loops per minute)')
        print(f'::: Sleep Rate: {self.sleep_rate:0.4f} secs \t(Sleeps {self.sleep_rate:0.4f} secs per msg)\n')
        print(f'RESULTING LOOP RATE: {self.loop_rate:0.4f} \t(Approx. {self.loop_rate:0.4f} secs per loop)\n')
        print(f'::: Euler Active: \t{self.euler_array}')
        print(f'::: Control Funcs:\t[{self.ctrl_func_array[0].__name__}  {self.ctrl_func_array[1].__name__}  {self.ctrl_func_array[2].__name__}]')
        print(f'::: Amplitude: \t\t{self.amplitude_array}')
        print(f'::: Period: \t\t{self.period_array}')
        print(f'::: Phase Shift: \t{self.phase_array}')
        print(f'::: Offset: \t\t{self.offset_array}')
        print(f'::: Repeat Loop: \t{self.loop_repeats} times\n')
        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')
        
        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def check_control_values(self):
        
        # Check if mode is valid
        if self.mode not in self.mode_list:
            raise ValueError(f"mode must be one of the following: {self.mode_list}\n")

        # Check if ctrl_function is valid
        for ctrl_function in range(len(self.ctrl_func_array)):
            if self.ctrl_func_array[ctrl_function] not in self.oscillating_functions:
                raise ValueError(f"ctrl_function at idx [{ctrl_function}] must be one of the following: {self.oscillating_functions}\n")

        # Check if publish_hz is valid
        if isinstance(self.publish_hz, int) == False or self.publish_hz > 1000 or self.publish_hz < 50:
            raise ValueError("publish_hz must be integer and within range [50, 1000]\n")
        
        # Check if sleep_override exists and is valid
        if self.sleep_override:
            if self.sleep_override > 0.1 or self.sleep_override < 0.001:
                raise ValueError("sleep_override must be within range [0.001, 0.1]\n")

        # Check if loop_repeats is valid
        if self.loop_repeats > 128 or self.loop_repeats < 1:
            raise ValueError("loop_repeats must be within range [1, 128]\n")

        # Check if loop_repeats is a multiple of max period
        if self.loop_repeats < max(self.period_array) or self.loop_repeats % max(self.period_array) != 0:
            print(f"WARNING: loop_repeats {self.loop_repeats} is not a multiple of max period {max(self.period_array)}")
            print(f"::: Robot may not complete full oscillation\n")

        # Check if sleep_rate is valid
        if self.sleep_rate < 0.002:
            print(f"WARNING: low sleep_rate of {self.sleep_rate} is not recommended. Might experience control lag") 
            print(f"::: Recommended sleep_rate >= 0.002. Decrease publish_hz to 500 or less or override sleep_rate\n")
            if self.sleep_rate < 0.001:
                raise ValueError("sleep_rate must be within range [0.001, 0.1]\n")

        # Check if euler is valid
        for euler in self.euler_array:
            if euler != 0 and euler != 1:
                raise ValueError("euler values must be 0 or 1\n")

        # Check if amplitude + offset is valid
        for index_012 in range(0,3):
            if abs(self.amplitude_array[index_012]) + abs(self.offset_array[index_012]) > 0.55:
                raise ValueError("sum of (amplitude + offset) must be within range [0, 0.55] for each euler angle\n")

        # Check if period is valid
        for period in self.period_array:
            if isinstance(period, float) == False or period > 4. or period < 1.:
                print(f'WARNING: period values should be specified within range [1., 4.] for continuous motion')
                if isinstance(period, float) == False or period > 16. or period < 1.:
                    raise ValueError("period values must be float within range [1., 16.] for each euler angle\n")

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

        # Check if resulting loop_rate is lower than 1 and warn user if it is 
        if self.loop_rate < 1.0:
            print(f'WARNING: Loop rate of {self.loop_rate} is less than 1 [1sec/loop]')
            print('::: Robot may move quickly and could damage itself\n')

        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def euler_control(self, mode='default', publish_hz=200, bpm=30, 
                                sleep_override=None, loop_repeats=8, 
                                euler_array=np.array([1, 1, 1]),
                                ctrl_func_array=np.array([math.sin, math.sin, math.sin]),
                                amplitude_array=np.array([0.5, 0.5, 0.4]),
                                offset_array=np.array([0.0, 0.0, 0.0]), 
                                period_array=np.array([2.0, 2.0, 2.0]), 
                                phase_array=np.array([0.0, 0.0, 0.0]), 
                                force=False,
                                dev_check=True):
        '''ARG RANGES:
        recall y = Asin(B(x + C)) + D   ///   A = amplitude, 2pi/B = period, C = phase shift, D = offset

        mode = ['default', 'rand_dance', '...'] ->   Makes robot do variations of the default control loop.
                                                default = robot will run specified euler angles on repeat
                                                rand_dance = robot will do random dance with random euler angles

        publish_hz = [50, 1000] (Hz - int) ->   Number of msgs being sent per loop. Smoother motion at higher hz but more comp expensive. 
                                                If robot acting undesirably, try decreasing this value. Recommended 100+ 
        
        set_bpm = [1, 60] (bpm - int) ->    Sets the bpm at which the loops of the robot execute. If nothing is set, the robot will move
                                            at a rate of one control loop per second -> 1 loop/sec = 1 beat per sec = 60BPM. 
                                            Any specified BPM greater than 100 will be halved until within range [0-100]. Recommended value: 50-100                                             

        sleep_override = [0.001, 0.1] (int) ->  Sets the number of seconds that each loop will sleep for. Setting this breaks the loop_rate=1 [1sec/loop]
                                                Smaller equals faster loops, but more comp expensive. If you have high publish_hz [200+], then you 
                                                can use this to slow down the loops to get smoother motion.
        
        loop_repeats = [1, 128] (int)-> Number of times to repeat the loop. Use 1 for one loop. 
                                        Recommended to choose numbers that are 2^n.

        euler_array = [0, 1] -> 1/True enables the sin_roll, sin_pitch, sin_yaw respectively. 0/False disables the sin_roll, sin_pitch, sin_yaw respectively.

        ctrl_func_array = [math.sin, udcf.neg_sin, ... ] -> Continuous functions from math or user_defined 'udcf' library.

        amplitude_array = [-0.55, 0.55] (radians - float) ->  Larger value equals bigger angle. Negative is reversed. Recommended 0.6 for full angle.
                                                            CAREFUL WITH THIS ONE.. dont max joint limits. Set skip_check=False first to check your code.
        
        offset_array = [-0.55, 0.55] (radians - float) -> Offsets the oscillation. Combination of offset and amplitude cannot exceed 0.7.
                                                        CAREFUL WITH THIS ONE.. dont max joint limits. Set skip_check=False first to check your code.

        period_array = [1, 16] (cycles - float) ->  quantity equals period of oscillation. Recommended 1 for one oscillation. 
                                                    Use 2 to get an oscillation which takes twice as long. Use 0 to get infinite oscillations? (j.k. dont do that)
        
        phase_array = [-2, 2] (radians - float) ->  Quantity equals phase shift of oscillation. Recommended 0 for no phase shift. 1 for half phase shift.
                                                    If you enter 2, it will be converted to 0. If you enter -2, it will be converted to 0.
                                                    For sin(), phase should be 0. For cos(), phase should be -1/4 of period. 
                                                    These values will auto-correct unless force=True. 

        force = [True, False] -> If True, will force the phase values to be used. If False, will auto-correct phase values to be used.

        dev_check = [True, None] -> If True, will check if all values are within range. If False, will skip all checks.
        
        printer = ['all', 'angles', None] -> If True, will print out the values of the control loop. If False, will not print out the values of the control loop.

        
        Note:   If you want a loop_rate of 1 as [1sec/loop] -> this is useful for making robot move to a given time signature..
                The following equation is used to calculate loop_rate: 
                
                loop_rate = sleep_rate * publish_hz
                sleep_rate = 1 / publish_hz

                This loop_rate will keep to 1 so long as sleep_override is not used.
                You can increase publish_hz and override the sleep_rate to get smoother motion while breaking loop rate.  
        '''

        # Calculate sleep_rate while taking into account bpm_override if it exists

        self.mode = mode
        self.publish_hz = publish_hz
        self.sleep_override = sleep_override
        self.loop_repeats = loop_repeats
        self.euler_array = euler_array
        self.ctrl_func_array = ctrl_func_array
        self.amplitude_array = amplitude_array
        self.offset_array = offset_array
        self.period_array = period_array
        self.phase_array = phase_array
        self.force = force
        self.dev_check = dev_check
        self.timestep = 0

        # Changes bpm and sets sleep_rate
        self.change_bpm(bpm)

        # Override sleep_rate if sleep_override is set
        if self.sleep_override:
            self.override_sleep_rate()

        # Check if all values are within range // skip ONLY if you are very confident in your code >:) 
        if self.dev_check:
            self.check_control_values()

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

                self.amplitude_array[0] * self.ctrl_func_array[0](2*math.pi / self.period_array[0] * ((self.timestep / self.publish_hz) + self.phase_array[0])) + self.offset_array[0],
                self.amplitude_array[1] * self.ctrl_func_array[1](2*math.pi / self.period_array[1] * ((self.timestep / self.publish_hz) + self.phase_array[1])) + self.offset_array[1],
                self.amplitude_array[2] * self.ctrl_func_array[2](2*math.pi / self.period_array[2] * ((self.timestep / self.publish_hz) + self.phase_array[2])) + self.offset_array[2]]),
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
                # After every 2nd loop, do one of four things:
                    # A: Change ctrl and euler angles
                    # B: Change ctrl and euler angles and reverse trajectory
                    # C: Reverse trajectory
                    # D: Change ctrl

                if self.timestep % (self.publish_hz * 2) == 0 and self.timestep != 0:
                    next_move = random.getrandbits(2)   # Randomly choose to change euler angles or reverse trajectory

                    if next_move == 0 or (self.timestep % (self.publish_hz * max(self.period_array)) and self.timestep != 0):
                        self.rand_ctrl_func_array()     # New ctrl functions
                        self.rand_euler()               # New euler angles and params
                        self.check_control_values()     # Checks values for safety
                        if self.printer == 'all':
                            self.status_printer()       

                    elif next_move == 1:
                        self.rand_ctrl_func_array()     # New ctrl functions
                        self.rand_euler()               # New euler angles and params
                        self.check_control_values()     # Checks values for safety
                        if self.printer == 'all':
                            self.status_printer()       
                        if self.printer:
                            print(f'>> Reverse Traject @ timestep: \t{self.timestep:05d}\n')
                        self.amplitude_array = self.amplitude_array * -1

                    elif next_move == 2:
                        if self.printer:
                            print(f'>> Reverse Traject @ timestep: \t{self.timestep:05d}\n')
                        self.amplitude_array = self.amplitude_array * -1

                    elif next_move == 3:
                        self.rand_ctrl_func_array()     # New ctrl functions


            remaining_delay = max(start + ((self.timestep+1) * self.sleep_rate) - time.time(), 0) # replace self.timestep with i where I starts at 1
            # print("remaining delay: %s" % remaining_delay)                                      # Uncomment to see remaining delay

            time.sleep(remaining_delay)        # Sleep for the remaining delay

        
        print(f'>> End of: euler_control in [{self.mode}] mode\n')        
        print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')

        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


### ToDo List:

# Outsource mode logic
#  remove rand_dance logic from master function
#  create other modes (e.g. push ups, dance_fast, dance_slow, pose) 

# Get the robot to dance to a song and film it

# Add a function to change any of the self values (BPM, amplitude, offset, period etc.) 
    # To do this you likely need the ability to have multiple functions running at once
    # This is where you need ROS


# Get the robot to walk in a line forward backwards 
# Get the robot to walk in a circle 
# Start doing motion such as walking in circle or line or spinning on the spot or jumping 

# Check: Maybe and ToDo 
###



### Old Code:
# # Set up the phase array based on the control function (sin or cos)
# if self.ctrl_func_array == math.cos:
#     for index_012 in range(0,3):
#         self.phase_array[index_012] = self.period_array[index_012] / -4.0

# # Check if phase is valid for sin() function
# if self.ctrl_func_array == math.sin and self.force==False:
#     for index_012 in range(0,3):
#         if self.phase_array[index_012] != 0.:
#             print(f'WARNING: phase value at idx[{index_012}] should be set to float [0.0] for sin() function')
#             print(f'::: Overriding phase[{index_012}] of {self.phase_array[index_012]} to: 0.0')
#             print(f"::: To force values, set 'force=True' \n")
#             self.phase_array[index_012] = 0


# # Check if phase is valid for cos() function
# if self.ctrl_func_array == math.cos:
#     for index_012 in range(0,3):
#         if self.phase_array[index_012] != self.period_array[index_012] / -4.0:
#             print(f'WARNING: phase value at idx[{index_012}] should be set to float [period/-4.0] for cos() function')   
#             print(f'::: Overriding phase[{index_012}] of {self.phase_array[index_012]} to: {self.period_array[index_012] / -4.0}')
#             print(f"::: To force values, set 'force=True' \n")
#             self.phase_array[index_012] = self.period_array[index_012] / -4.0