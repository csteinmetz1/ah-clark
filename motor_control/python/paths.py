import numpy as np
import matplotlib.pyplot as plt

with open("path.txt") as path_fp:
	path = path_fp.readlines()

fig, ax = plt.subplots()

for point in path:
	x, y = point.split()
	ax.plot(float(x), float(y), marker = 'o', color='b')

plt.gca().set_aspect('equal', adjustable='box')
ax.set_xlim(0, 66)
ax.set_ylim(0, 60)
ax.grid('on')
plt.show()
