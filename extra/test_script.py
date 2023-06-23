import math

def sin_with_parameters(x, amplitude=1.0, phase_shift=0.0, period=2*math.pi, offset=0.0):
    """
    Computes the value of a sine function with adjustable parameters.
    
    Args:
        x (float): The input value.
        amplitude (float): The amplitude of the sine wave (default: 1.0).
        phase_shift (float): The phase shift of the sine wave in radians (default: 0.0).
        period (float): The period of the sine wave in radians (default: 2*pi).
        offset (float): The vertical offset of the sine wave (default: 0.0).
    
    Returns:
        float: The computed value of the sine function.
    """
    return amplitude * math.sin((x + phase_shift) * 2*math.pi / period) + offset

# Example usage:
x = 0.0
amplitude = 2.0
phase_shift = 2* math.pi 
period = 2*math.pi
offset = 0

result = sin_with_parameters(x, amplitude, phase_shift, period, offset)
print(result)