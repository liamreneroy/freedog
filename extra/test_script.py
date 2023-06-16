import time
import math

def roll_shake(publish_hz=100, granularity=1000, amplitude=0.5, frequency=1, reversed=False):
    '''ARG RANGES:
    publish_hz = [1, 500] (Hz) -> smaller equals faster execution time. If robot acting weird, try decreasing this value. Recommended 100+ 
    granularity = [500, 5000] (cycles) -> smaller equals slightly faster but choppier execution. Recommended 500+ for smooth motion.
    amplitude = [0, 0.5] (radians) -> smaller equals less roll angle. Recommended 0.5 for full roll angle.
    frequency = [1, 3] (cycles) -> quantity equals number of oscillations. Recommended 1 for one oscillation.
    reversed = [True, False] -> True reverses the direction of the roll_shake
    '''

    # Args Safety-Check
    # Note: Granularity divided by publish_hz equals seconds approximately)

    if isinstance(publish_hz, int) == False or publish_hz > 500 or publish_hz < 50:
        raise ValueError("publish_hz must be integer and within range [50, 500]")
    
    if granularity % 10 != 0 or granularity > 5000 or granularity < 500:
        raise ValueError("granularity must be integer, divisible by 10 and within range [500, 5000]")
    
    if isinstance(amplitude, float) == False or amplitude > 0.5 or amplitude < 0:
        raise ValueError("amplitude must be float and within range [0, 0.5]")

    if isinstance(frequency, int) == False or frequency > 3:
        raise ValueError("frequency must be integer and within range [1, 3]")

    if reversed == True: # Reverse the direction of the roll
        amplitude = -amplitude


    print('roll_shake ::: Publishing at {}Hz'.format(publish_hz))
    for timestep in range(0, granularity):  # Not sure if I need the +1 but this makes it start and end at zero
        time.sleep(float(1./publish_hz))    # Average  --> Sleep to achieve desired 0.002 time interval (in seconds)

 
        roll_angle = [0, amplitude * math.sin(frequency * timestep / granularity * ((math.pi)*2)), 0]


        if timestep % 10 == 0: #Print every 10 cycles
            print(f"TimeStep: {timestep} ,  Roll Angle: {roll_angle}\n")


roll_shake()