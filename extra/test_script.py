import numpy as np

pear = 2 / -4.0
print(pear)


phase_array = np.array([-3., -3., -3.])
period_array = np.array([2, 1, 1])


for index_012 in range(0,3):
    print("index_012: ", index_012)
    print(period_array[index_012])
    phase_array[index_012] = period_array[index_012] / -4.0

print(phase_array)