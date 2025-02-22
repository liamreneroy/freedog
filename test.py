import math

param_dict = 'object'

# BODY_DIRECTION (yaw angle of the robots whole body)
if param_dict == 'user':
    conv_param_dict = 0.0
elif param_dict == 'object':
    conv_param_dict= 90 * math.pi / 180

print("conv_param_dict: ", conv_param_dict)