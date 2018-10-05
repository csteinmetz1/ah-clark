import sys
import numpy as np
import matplotlib.pyplot as plt
from tiva import TivaController

Tiva = TivaController(units_per_cm=1, arm1_cm=20, arm2_cm=10, 
						x_offset_cm=10, y_offset_cm=-5, bufsize=8)

# draw a circle
n_steps = 50
r = 5
u = np.linspace(0, 2*np.pi, num=n_steps)
x = r * np.cos(u) + 3
y = r * np.sin(u) + 18

#x = [5]
#y = [10]

fig, ax = plt.subplots()

for idx, point in enumerate(zip(x, y)):
	Tiva.move_arm(point[0], point[1])

	sys.stdout.write("Computing step {0}/{1}\r".format(idx, n_steps))
	sys.stdout.flush()

	x1, y1 = [Tiva.x_offset, Tiva.x1], [Tiva.y_offset, Tiva.y1]
	#x1, y1 = [0, Tiva.x1], [0, Tiva.y1]
	x2, y2 = [Tiva.x1, Tiva.x2], [Tiva.y1, Tiva.y2]
	ax.plot(x1, y1, marker = 'o', color='b')
	ax.plot(x2, y2, marker = 'o', color='b')
	ax.plot(point[0], point[1], marker = 'o', color='r')
	plt.axis('equal')
	#ax.set_xlim(0, 100)
	#ax.set_ylim(0, 300)
	ax.grid('on')
	plt.savefig("img/{}.png".format(idx))
	
	#comm.send([q1, q2])
