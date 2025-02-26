import math
import numpy as np

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


# Oscillating control functions
def neg_sin(x):
    return -math.sin(x)

def abs_sin(x):
    return abs(math.sin(x))

def neg_abs_sin(x):
    return -abs(math.sin(x))

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Array functions
def reverse_array(original_array):
    reversed_array = original_array[::-1]
    return reversed_array

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~