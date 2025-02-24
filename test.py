import math

import numpy as np


action_space = np.empty((2, 2, 3), dtype=object)

# Fill each element with a new empty list
for idx in np.ndindex(action_space.shape):
    action_space[idx] = []

print(action_space)


motion_parameters = {
    "0": ["user", "object"],
    "1": ["left", "neutral", "right"],
    "2": ["backwards.", "neutral", "forward"],
    "3": ["low", "neutral", "high"],
    "4": ["smooth", "shaky"],
    "5": ["slow", "medium", "fast"]
    }



print(motion_parameters[str(1)][1])
