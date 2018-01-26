import matplotlib.pyplot as plt
import numpy as np
import os
import sys
fileName = sys.argv[1]
x = []
y = []

with open(fileName) as f:
    for line in f:
        line = line.split(',')
        x.append(line[0])
        y.append(line[1])

plt.plot(x, y)
plt.show()
