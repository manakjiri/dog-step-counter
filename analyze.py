import sys
import matplotlib.pyplot as plt
import numpy as np

file_path = sys.argv[1]

def hex2int(s: str, bytes=4):
    bits = bytes * 8
    value = int(s, 16)
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value


gyro_vals = []

with open(file_path, 'r') as file:
    file.readline() # skip the first line
    for line in file.readlines():
        line = line.strip()
        if line:
            if '=' in line:
                print(line)
            
            else:
                acc, temp, gyro = line.split(';')
                acc = list(map(hex2int, acc.split(',')))
                temp = float(temp)
                gyro = list(map(hex2int, gyro.split(',')))

                #print(acc, temp, gyro)
                gyro_vals.append(gyro)


gyro_vals = np.array(gyro_vals)

plt.plot(gyro_vals[:,2], 'g')
plt.grid()
plt.show()

