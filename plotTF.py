#!/usr/bin/python
# Stephen Smith
# 10-12-2015

import sys
import csv
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import numpy as np
import time
from math import sqrt

###################################################################################################
### Serves to test output in csv format															###
###################################################################################################
def main(args):
	inputFile = args[0]
	makeMovie = True
	try:
		if args[1] == '-m':
			makeMovie = True
	except:
		pass
	x = []
	y = []
	z = []
	t = []
	with open(inputFile, 'rb') as csvFile:
		csvReader = csv.DictReader(csvFile)
		for row in csvReader:
			# x.append(float(row['T[3]']))
			# y.append(float(row['T[7]']))
			# z.append(float(row['T[11]']))
			# t.append(float(row['time']))
			x.append(float(row['x']))
			y.append(float(row['y']))
			z.append(float(row['z']))
			t.append(0)

	def getData(num,data,line):
		plt.title(t[num])
		line.set_data(data[0:2,:num])
		line.set_3d_properties(data[2,:num])
		return line

	steps = len(t)
	x = np.array([x]).T
	y = np.array([y]).T
	z = np.array([z]).T
	t = np.array([t]).T
	data = np.concatenate((x,y,z),axis=1).T

	print 'Limits: ', min(x), max(x), min(y), max(y), min(z), max(z)
	
	fig1 = plt.figure()
	ax = p3.Axes3D(fig1)
	# plt.axis([min(x), max(x), min(y), max(y)])
	ax.set_xlim3d([min(x),max(x)])
	ax.set_ylim3d([min(y),max(y)])
	ax.set_zlim3d([min(z),max(z)])
	l, = plt.plot(data[0,0:1],data[1,0:1],data[2,0:1])
	lineAnimation = animation.FuncAnimation(fig1, getData, steps, fargs=(data,l), interval=40)	

	if makeMovie:
		lineAnimation.save('factorGraph.mp4',writer='avconv')

	plt.show()

if __name__ == "__main__":
	main(sys.argv[1:])