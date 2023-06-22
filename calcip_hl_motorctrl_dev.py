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
random.seed(55) # random.seed(time.time())  eventually use this


import numpy as np
np.set_printoptions(formatter={'float': lambda x: '%7.4f' % (x)})
# np.set_printoptions(precision=3)



class MotorControl:

    def __init__(self):
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

        print(">> MotorControl Initialized\n")        

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def parse_data(self, print_out=False):
        ''' Parse data from the robot and print out the first packet'''
        print(">> Parsing Data\n")        

        data = self.conn.getData()

        paket_count = 0
        for paket in data:
            self.hstate.parseData(paket)

            if print_out == True and paket_count == 0:
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
        ''' Recover control and put robot into standing pose'''
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
        ''' Terminate control and break connection '''
        print(">> Executing: FORCE_STAND\n")            # Put robot back into neutral standing state
        self.hcmd.mode = MotorModeHigh.FORCE_STAND
        self.hcmd.euler = [0, 0, 0]
        self.hcmd.bodyHeight = 0.0
        cmd_bytes = self.hcmd.buildCmd(debug=False)     # Build the stand command
        self.conn.send(cmd_bytes)                       # Send the stand command
        time.sleep(1)                                   # Sleep for 1 second

        print(">> Executing: STAND_DOWN\n")
        self.hcmd.mode = MotorModeHigh.STAND_DOWN
        cmd_bytes = self.hcmd.buildCmd(debug=False)     # Build the lay-down command
        self.conn.send(cmd_bytes)                       # Send the lay-down command
        time.sleep(1)                                   # Sleep for 1 second

        print(">> Executing: IDLE\n")
        self.hcmd.mode = MotorModeHigh.IDLE
        cmd_bytes = self.hcmd.buildCmd(debug=False)     # Build the idle command
        self.conn.send(cmd_bytes)                       # Send the idle command
        time.sleep(0.5)                                 # Sleep for 0.5 second

        print(">> Executing: DAMPING\n")
        self.hcmd.mode = MotorModeHigh.DAMPING
        cmd_bytes = self.hcmd.buildCmd(debug=False)  # Build the final command
        self.conn.send(cmd_bytes)                    # Send the final command
        time.sleep(1)                           # Sleep for 1 second
        
        return True                             # Break connection at end

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
        # Calculate loop_rate (should be 1 if sleep_override is not used)
        self.loop_rate = self.sleep_rate * self.publish_hz
        print(f"::: New loop_rate: {self.loop_rate}\n")
        return
    
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def rand_rollpitchyaw(self):
        ''' Randomize rollpitchyaw_array and amplitude_array ''' # could also use randint(-1,1) to et -1, 0, 1 and remove rollpitchyaw_array
        self.rollpitchyaw_array = np.array([random.getrandbits(1), random.getrandbits(1), random.getrandbits(1)])
        while self.rollpitchyaw_array.sum() == 0:
            self.rollpitchyaw_array = np.array([random.getrandbits(1), random.getrandbits(1), random.getrandbits(1)])
        
        self.amplitude_array = np.array([random.uniform(0.3, 0.5) * random.choice([-1, 1]), 
                                         random.uniform(0.3, 0.5) * random.choice([-1, 1]),
                                         random.uniform(0.3, 0.4) * random.choice([-1, 1])])
        
        self.period_array=np.array([random.choice([0.5, 1]), 
                                    random.choice([0.5, 1]), 
                                    random.choice([0.5, 1])])

        return
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def rand_ctrl_function(self):
        # Sort through this to only take continuous functions
        oscillating_functions = [math.sin, math.cos]
        self.ctrl_function = random.choice(oscillating_functions)
        print(f"::: New ctrl_function: {self.ctrl_function.__name__}\n")
        return
        
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def change_bpm(self, new_bpm):

        # Check if new bpm is valid 
        if new_bpm <= 0:
            print(f"WARNING: Attempt to change BPM to negative value {new_bpm} is not allowed")
            if self.bpm:
                print(f"::: BPM remains at: {self.bpm}\n")
                return
            
            elif self.bpm == None:
                self.bpm = 60

        else:
            self.bpm = new_bpm
        
        while self.bpm > 60:   # Sets max BPM range [0, 60] so robot cant move faster than 1 loop/sec
            self.bpm /= 2 

        bpm_to_bps_factored = (60. / self.bpm)              
        self.sleep_rate = (1 / int(self.publish_hz)) * bpm_to_bps_factored 
        
        if self.printer:
            print(f">> BPM set to: {self.bpm}\n")

        self.calculate_loop_rate()

        return
    
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def status_printer(self):
            print('\n+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
            print(f'>> Executing: rollpitchyaw @ timestep: {self.timestep}')
            print(f'::: Control Function: {self.ctrl_function.__name__} \t(funct which dictates motion trajectory)')
            print(f'::: Publish Rate: {self.publish_hz} hz \t({self.publish_hz} messages per loop)')
            print(f'::: Beats Per Min: {self.bpm} bpm \t({self.bpm} loops per minute)')
            print(f'::: Sleep Rate: {self.sleep_rate} secs \t(Sleeps {self.sleep_rate} secs per msg)\n')
            print(f'RESULTING LOOP RATE: {self.loop_rate} \t(Approx. {self.loop_rate} second per loop)\n')
            print(f'::: RollPitchYaw: \t{self.rollpitchyaw_array}')
            print(f'::: Amplitude: \t\t{self.amplitude_array}')
            print(f'::: Period: \t\t{self.period_array}')
            print(f'::: Phase Shift: \t{self.phase_array}')
            print(f'::: Offset: \t\t{self.offset_array}')
            print(f'::: Repeat Loop: \t{self.loop_repeats} times\n')
            print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def check_control_values(self):
        # Check if granularity_override exists and is valid
        if self.sleep_override:
            if self.sleep_override > 0.1 or self.sleep_override < 0.001:
                raise ValueError("sleep_override must be within range [0.001, 0.1]\n")
            
        # Check if period is valid
        for period in self.period_array:
            if period > 10 or period < 1./3:
                raise ValueError("period values must be within range [1/3, 10] for each euler angle\n")

        # Check if period and publish_hz combo is beyond safety limiter and override publish_hz if necessary 
        for period in self.period_array:
            if period < 0.5 and self.publish_hz > 250:
                print(f"WARNING: Combination of low period {period} and publish_hz {self.publish_hz} is beyond safety limiter")
                self.publish_hz = 250
                self.sleep_rate = 1 / int(self.publish_hz)
                print(f"::: Overriding publish_hz to: {self.publish_hz}\n") 

        # Check if publish_hz is valid
        if isinstance(self.publish_hz, int) == False or self.publish_hz > 1000 or self.publish_hz < 50:
            raise ValueError("publish_hz must be integer and within range [50, 1000]\n")
        
        # Check if sleep_rate is valid
        if self.sleep_rate < 0.002:
            print(f"WARNING: low sleep_rate of {self.sleep_rate} is not recommended. Might experience control lag") 
            print(f"::: Recommended sleep_rate >= 0.002. Decrease publish_hz to 500 or less or override sleep_rate\n")
            if self.sleep_rate < 0.001:
                raise ValueError("sleep_rate must be within range [0.001, 0.1]\n")

        # Check if amplitude + offset is valid
        for index_123 in range(0,3):
            if abs(self.amplitude_array[index_123]) + abs(self.offset_array[index_123]) > 0.7:
                raise ValueError("sum of (amplitude + offset) must be within range [0, 0.7] for each euler angle\n")

        # Check if phase is valid
        for phase in self.phase_array:
            if phase > 2*math.pi or phase < -2*math.pi:
                print(f'WARNING: phase values should be specified within range [-2pi, 2pi] for each euler angle\n')

        # Check if loop_repeats is lower than 1
        if self.loop_rate < 1:
            print(f'WARNING: Loop rate of {self.loop_rate} is less than 1 [1sec/loop]')
            print('::: Robot may move quickly and could damage itself\n')

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def rollpitchyaw_control(self, ctrl_function=math.sin, publish_hz=200, bpm=60, 
                            sleep_override=None, loop_repeats=2, 
                            rollpitchyaw_array=np.array([1, 1, 1]),
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
        
        set_bpm = [1, 60] (bpm - integer) -> Sets the bpm at which the loops of the robot execute. If nothing is set, the robot will move
                                             at a rate of one control loop per second -> 1 loop/sec = 1 beat per sec = 60BPM. 
                                             Any specified BPM greater than 100 will be halved until within range [0-100]. Recommended value: 50-100                                             

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

        # Calculate sleep_rate while taking into account bpm_override if it exists

        self.ctrl_function = ctrl_function
        self.publish_hz = publish_hz
        self.sleep_override = sleep_override
        self.loop_repeats = loop_repeats
        self.rollpitchyaw_array = rollpitchyaw_array
        self.amplitude_array = amplitude_array
        self.offset_array = offset_array
        self.period_array = period_array
        self.phase_array = phase_array
        self.dev_check = dev_check
        self.printer = printer
        self.timestep = None


        self.change_bpm(bpm)

        # Override sleep_rate if sleep_override is set
        if self.sleep_override:
            self.override_sleep_rate()

        # Check if all values are within range // skip ONLY if you are very confident in your code >:) 
        if self.dev_check:
            self.check_control_values()

        if self.dev_check or self.printer:
            self.status_printer()

        if self.dev_check:
            print("Check set values and hit 'enter' to execute. 'ctrl + c' to abort.")
            input()

        if self.printer:
            a = datetime.datetime.now()

        start = time.time()

        i = 0
        for self.timestep in range(0, self.publish_hz*self.loop_repeats):
            i += 1
            
            # Set highCmd values 
            self.hcmd.mode = MotorModeHigh.FORCE_STAND   
            self.hcmd.euler = np.multiply(np.array([
                self.amplitude_array[0] * self.ctrl_function(((math.pi)*2 / self.period_array[0]) * ((self.timestep / self.publish_hz) + self.phase_array[0])) + self.offset_array[0],
                self.amplitude_array[1] * self.ctrl_function(((math.pi)*2 / self.period_array[1]) * ((self.timestep / self.publish_hz) + self.phase_array[1])) + self.offset_array[1],
                self.amplitude_array[2] * self.ctrl_function(((math.pi)*2 / self.period_array[2]) * ((self.timestep / self.publish_hz) + self.phase_array[2])) + self.offset_array[2]]),
                self.rollpitchyaw_array)
            
            if self.timestep % 10 == 0 and self.printer:                  # Print every 10 cycles
                if self.timestep % self.publish_hz == 0 and self.printer:      # Print once per loop to check loop rate (should be 1.0 secs)
                    b = datetime.datetime.now()     
                    c = b - a
                    a = datetime.datetime.now()
                    print(f'TimeStep: {self.timestep:04d} \t  Euler RPY Angle: {self.hcmd.euler} \t LoopRate: {c.total_seconds()} secs\n')
                    continue
                print(f'TimeStep: {self.timestep:04d} \t  Euler RPY Angle: {self.hcmd.euler}\n')  # Comment out if you dont want to see every timestep

            cmd_bytes = self.hcmd.buildCmd(debug=False)  # Build the command
            self.conn.send(cmd_bytes)                    # Send the command

            remaining_delay = max(start + (i * self.sleep_rate) - time.time(), 0)
            # print("remaining delay: %s" % remaining_delay) # Uncomment to see remaining delay

            time.sleep(remaining_delay)        # Sleep for the remaining delay

        print('\n+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')
        return
    
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def rollpitchyaw_dance(self, ctrl_function=math.sin, publish_hz=200, bpm=30, 
                                sleep_override=None, loop_repeats=10, 
                                rollpitchyaw_array=np.array([1, 1, 0]),
                                amplitude_array=np.array([0.5, 0.5, 0.4]),
                                offset_array=np.array([0, 0, 0]), 
                                period_array=np.array([1, 0.5, 1]), 
                                phase_array=np.array([0, 0, 0]), 
                                dev_check=True,
                                printer=True):



        '''ARG RANGES:
        recall y = Asin(B(x + C)) + D   ///   A = amplitude, 2pi/B = period, C = phase shift, D = offset

        ctrl_function = [math.sin, math.cos ... ] -> Any continuous function from math library.

        publish_hz = [50, 1000] (Hz - integer) -> Number of msgs being sent per loop. Smoother motion at higher hz but more comp expensive. 
                                                If robot acting undesirably, try decreasing this value. Recommended 100+ 
        
        set_bpm = [1, 60] (bpm - integer) -> Sets the bpm at which the loops of the robot execute. If nothing is set, the robot will move
                                             at a rate of one control loop per second -> 1 loop/sec = 1 beat per sec = 60BPM. 
                                             Any specified BPM greater than 100 will be halved until within range [0-100]. Recommended value: 50-100                                             

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

        # Calculate sleep_rate while taking into account bpm_override if it exists

        self.ctrl_function = ctrl_function
        self.publish_hz = publish_hz
        self.sleep_override = sleep_override
        self.loop_repeats = loop_repeats
        self.rollpitchyaw_array = rollpitchyaw_array
        self.amplitude_array = amplitude_array
        self.offset_array = offset_array
        self.period_array = period_array
        self.phase_array = phase_array
        self.dev_check = dev_check
        self.printer = printer
        self.timestep = None

        self.change_bpm(bpm)

        # Override sleep_rate if sleep_override is set
        if self.sleep_override:
            self.override_sleep_rate()

        # Check if all values are within range // skip ONLY if you are very confident in your code >:) 
        if self.dev_check:
            self.check_control_values()

        if self.dev_check or self.printer:
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
                self.amplitude_array[0] * self.ctrl_function(((math.pi)*2 / self.period_array[0]) * ((self.timestep / self.publish_hz) + self.phase_array[0])) + self.offset_array[0],
                self.amplitude_array[1] * self.ctrl_function(((math.pi)*2 / self.period_array[1]) * ((self.timestep / self.publish_hz) + self.phase_array[1])) + self.offset_array[1],
                self.amplitude_array[2] * self.ctrl_function(((math.pi)*2 / self.period_array[2]) * ((self.timestep / self.publish_hz) + self.phase_array[2])) + self.offset_array[2]]),
                self.rollpitchyaw_array)

            if self.timestep % self.publish_hz == 0 and self.printer:        # Print once per loop to check loop rate (should be 1.0 secs)
                b = datetime.datetime.now()     
                c = b - a
                a = datetime.datetime.now()
                print(f'TimeStep: {self.timestep:04d} \t  Euler RPY Angle: {self.hcmd.euler} \t LoopRate: {c.total_seconds()} secs\n')
                
            elif self.timestep % 10 == 0 and self.printer:                    # Print every 10 timesteps
                print(f'TimeStep: {self.timestep:04d} \t  Euler RPY Angle: {self.hcmd.euler}\n')  # Comment out if you dont want to see every timestep

            cmd_bytes = self.hcmd.buildCmd(debug=False)  # Build the command
            self.conn.send(cmd_bytes)                    # Send the command

            # After every loop, perform the reverse
            if self.timestep % self.publish_hz == 0 and self.timestep != 0:
                print(f'>> Reverse Traject @ timestep: {self.timestep}\n')
                self.amplitude_array = self.amplitude_array * -1

                # After every 2nd loop, perform a new random loop configuration
                if self.timestep % (self.publish_hz * 2) == 0 and self.timestep != 0:
                    print(f'>> New Traject @ timestep: {self.timestep}\n')
                    self.rand_rollpitchyaw()
                    self.check_control_values()
                    self.status_printer()

            remaining_delay = max(start + ((self.timestep+1) * self.sleep_rate) - time.time(), 0) # replace self.timestep with i where I starts at 1
            # print("remaining delay: %s" % remaining_delay) # Uncomment to see remaining delay

            time.sleep(remaining_delay)        # Sleep for the remaining delay

        print('\n+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n')
        return

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~











### ToDo:
# Finish rand function
# Try integrating random function into sin_rpy main loop to change motion randomly
# Try integrating change_bpm function into sin_rpy main loop to see if you can change the bpm 
# See if you can assign sin() or cos() to as an object.attribute so you can call either one
# Get the robot to dance to a song and film it

# Add an absolute value shifter [-1, 0, 1] but this might make the motion choppy at point of shift
# Add a function to change any of the self values (amplitude, offset, period, phase, publish_hz, sleep_override, loop_repeats)
# Create a function that can be used to create a smooth transition between two sin functions

# Get the robot to walk in a line forward backwards 
# Get the robot to walk in a circle 
# Start doing motion such as walking in circle or line or spinning on the spot or jumping 
###
