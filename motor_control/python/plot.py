import sys
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
from tiva import TivaController

Tiva = TivaController(units_per_cm=1, arm1_cm=45, arm2_cm=20, 
						x_offset_cm=33, y_offset_cm=-10, bufsize=8)

n_steps = 50
plot = 'point'
clear = True

if   plot == 'random':
	x = np.random.randint(10, 60, n_steps)
	y = np.random.randint(20, 50, n_steps)
elif plot == 'point':
	x = [5]
	y = [20]
elif plot == 'line':
	x = np.linspace(0, 15, n_steps/2) 
	y = np.linspace(5, 20, n_steps)
elif plot == 'circle':
	r = 15
	u = np.linspace(0, np.pi,  num=n_steps)
	x = r * np.cos(u) + 30
	y = r * np.sin(u) + 30
elif plot == 'weird':	
	t = np.linspace(15, 50,  num=n_steps)
	x = (t + 2 * np.sin(2 * t)) * 2
	y =  12 * np.sin(t) + 30

fig, ax = plt.subplots()

for idx, point in enumerate(zip(x, y)):

	sel = np.random.random_integers(0, 1)

	# move the arm
	Tiva.move_arm(point[0], point[1], negative=True)

	sys.stdout.write("Computing step {0}/{1}\r".format(idx, n_steps))
	sys.stdout.flush()

	# optionally clear the plot of past positions
	if clear:
		plt.cla()

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
	
	print(Tiva.q1)

	#Tiva.send([Tiva.q1, Tiva.q2]) # send signal to the Tiva
	#print(Tiva.receive())
	#sleep(1)
