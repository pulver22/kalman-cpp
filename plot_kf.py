import numpy as np
import matplotlib.pyplot as plt 

# Load data from disk
f_measurements = open("/home/pulver/Desktop/measurements.txt", "r")
measurements = []
for line in f_measurements:
    measurements.append(float(line))
measurements = np.array(measurements)
# Crete a numpy array alternating each line in the file
measurements_arr = np.zeros(shape=(int(measurements.shape[0]/2), 2))
measurements_arr[:, 0] = measurements[0::2]
measurements_arr[:, 1] = measurements[1::2]


f_filter = open("/home/pulver/Desktop/filtered.txt", "r")
filtered = []
for line in f_filter:
    filtered.append(float(line))
filtered = np.array(filtered)
# Crete a numpy array alternating each line in the file
filtered_arr = np.zeros(shape=(int(filtered.shape[0]/2), 2))
filtered_arr[:, 0] = filtered[0::2]
filtered_arr[:, 1] = filtered[1::2]

# Plot measurements vs filtered
plt.plot(measurements_arr[:,0], measurements_arr[:,1], color='b', label='measurement')
plt.plot(filtered_arr[:,0], filtered_arr[:,1], color='r', label='filtered')
plt.legend()
plt.show()