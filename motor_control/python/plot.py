import sys
import numpy as np
import matplotlib.pyplot as plt
from tiva import TivaController

Tiva = TivaController(units_per_cm=1, arm1_cm=45, arm2_cm=20, 
						x_offset_cm=33, y_offset_cm=-10, bufsize=8)

# draw a circle
n_steps = 50
r = 15
u = np.linspace(0, np.pi,  num=n_steps)
x = r * np.cos(u) + 30
y = r * np.sin(u) + 30

#x = np.linspace(0, 15, n_steps/2) 
#x = np.concatenate((x, np.linspace(7, 9, n_steps/2)))
#y = np.linspace(5, 20, n_steps)

x = np.random.randint(10, 60, n_steps)
y = np.random.randint(20, 50, n_steps)

fig, ax = plt.subplots()

for idx, point in enumerate(zip(x, y)):
	Tiva.move_arm(point[0], point[1])

	sys.stdout.write("Computing step {0}/{1}\r".format(idx, n_steps))
	sys.stdout.flush()

	x1, y1 = [Tiva.x_offset, Tiva.x1], [Tiva.y_offset, Tiva.y1]
	x2, y2 = [Tiva.x1, Tiva.x2], [Tiva.y1, Tiva.y2]
	ax.plot(x1, y1, marker = 'o', color='b')
	ax.plot(x2, y2, marker = 'o', color='b')
	ax.plot(point[0], point[1], marker = 'o', color='r')
	plt.gca().set_aspect('equal', adjustable='box')
	ax.set_xlim(0, 66)
	ax.set_ylim(0, 134)
	ax.grid('on')
	plt.savefig("img/{}.png".format(idx))
	
	#comm.send([q1, q2])
