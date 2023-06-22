import random
import math
import numpy as np

random.seed(1)

amplitude_array = [random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), random.uniform(-0.4, 0.4)]
print(amplitude_array)


rollpitchyaw_array = np.array([random.getrandbits(1), random.getrandbits(1), random.getrandbits(1)])
print('before loop:', rollpitchyaw_array)
while rollpitchyaw_array.sum() == 0:
    rollpitchyaw_array = np.array([random.getrandbits(1), random.getrandbits(1), random.getrandbits(1)])
    print('in loop:', rollpitchyaw_array) 

