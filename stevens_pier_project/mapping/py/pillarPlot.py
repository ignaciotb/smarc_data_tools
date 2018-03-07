import matplotlib.pyplot as plt
import numpy as np
import math
import pickle
import pylab
import random
import os.path
import csv
from pylab import figure, show, rand
from matplotlib.patches import Ellipse, Circle
from scipy.stats import chi2
from scipy.optimize import curve_fit
# 
file = 'pillar00-new'
file2 = 'pillar-test'
saveName = 'hmmm'
showPath = False
showGrid = True

tableau20 = [(31, 119, 180), (174, 199, 232), (255, 127, 14), (255, 187, 120),  
             (44, 160, 44), (152, 223, 138), (214, 39, 40), (255, 152, 150),  
             (148, 103, 189), (197, 176, 213), (140, 86, 75), (196, 156, 148),  
             (227, 119, 194), (247, 182, 210), (127, 127, 127), (199, 199, 199),  
             (188, 189, 34), (219, 219, 141), (23, 190, 207), (158, 218, 229)]  
   
# Scale the RGB values to the [0, 1] range, which is the format matplotlib accepts.  
for i in range(len(tableau20)):  
    r, g, b = tableau20[i]  
    tableau20[i] = (r / 255., g / 255., b / 255.)

def rad2deg(rad):
	return rad/math.pi*180.0

def confidenceEllipsFromPillar(pillar):
	pos = np.array(pillar).T
	covar = np.cov(pos)
	w,v = np.linalg.eig(covar)
	if w[0] < w[1]:
		w[0], w[1] = w[1], w[0]
		v[0], v[1] = v[1], v[0]
	alpha = -rad2deg(math.atan2(v[0][1], v[0][0]))
	size = [2*math.sqrt(5.991*x) for x in w]
	return [size[0], size[1], alpha]

def ellipseFromConfidence(xy, para):
	ell = Ellipse(xy, para[0], para[1], para[2])
	ell.set_clip_box(ax.bbox)
	ell.set_alpha(0.2)
	ell.set_facecolor(tableau20[0])
	return ell

def rmsPillar(pillar):
	arr = np.array(pillar).T
	arr = arr - arr.mean(axis=1).reshape(2,1)
	return math.sqrt(np.power(arr,2).sum(axis=0).mean())

def circleFromRMS(xy, rad):
	cir = Circle(xy, rad)
	cir.set_clip_box(ax.bbox)
	cir.set_alpha(0.2)
	cir.set_facecolor(tableau20[0])
	return cir

def meanPillar(pillar):
	return np.array(pillar).T.mean(axis=1)

	# return [sum(x)/float(len(x)) for x in zip(*pillar)]

def distToLine(A,B,C,x,y):
	return abs(A*x+B*y+C)/math.sqrt(A**2+B**2)




current_directory = os.path.dirname(__file__)
parent_directory = os.path.split(current_directory)[0]
file_path = os.path.join(parent_directory, 'savedData/')
f = open(file_path + file + '.txt', 'rw')

csv_out = open(file_path + file + '.csv', 'wb')
csvf = csv.writer(csv_out)
pillars = []
features = []
mode = 0
for line in f:
	if "PILLARS" in line:
		mode = 0
		continue
	elif "FEATURES" in line:
		mode = 1
		continue

	if mode == 0:
		if "Pillar" in line:
			pillars.append([])
		else:
			tmpList = line.rstrip().split(',')
			print tmpList
			pillars[-1].append([float(tmpList[0]), -float(tmpList[1])])
	elif mode == 1:
		if "Feature" in line:
			features.append([])
		else:
			tmpList = line.rstrip().split(',')
			print tmpList
			features[-1].append([float(tmpList[0]), -float(tmpList[1])])
	# print("a", line)
# print pillars


stat = []
	
fig = figure()
ax = fig.add_subplot(111, aspect='equal')
for i in range(len(pillars)):
	plt.scatter(*zip(*pillars[i]), color=tableau20[i+3%20], marker="o", s=50)
	# plt.scatter(*zip(*features[i]), color=tableau20[i+3%20], marker="x", s=20, alpha=0.3)


meanPillars = []
for pillar in pillars:
	l = meanPillar(pillar)
	# print l
	meanPillars.append(l)
	stat.append([rmsPillar(pillar)])
	stat[-1].extend(confidenceEllipsFromPillar(pillar)[0:2])
	plt.scatter(l[0], l[1], s=40)
	ax.add_artist(ellipseFromConfidence(l,confidenceEllipsFromPillar(pillar)))
	ax.add_artist(circleFromRMS(l, rmsPillar(pillar)))
plt.gca().set_aspect('equal', adjustable='box')

# csvf.writerow(["Pillar" ,"Hits", "RMS (m)", "Ellipse x(m)", "Ellipse y(m)"])
for i in range(len(stat)):
	print  '{}{}{}'.format("pillar ", i+1,":"), '{}{}'.format("# ", len(pillars[i])),  '{}{}'.format("rms: ", stat[i][0]), '{}{}'.format("ellipse: ", stat[i][1:3])
	# print  '{}'.format(len(pillars[i])),  '{}{}'.format(",", stat[i][0]), '{}{}{}{}'.format(",", stat[i][1], ",", stat[i][2])
	csvf.writerow([str(i+1) ,len(pillars[i]), stat[i][0], stat[i][1], stat[i][2]])

# Find average values for statistics
meanStat = np.array(stat).T.mean(axis=1)

rms = 0;
ellipse1 = 0
ellipse2 = 0
pillarcount = 0
for i in range(0,9):
	# print stat[i][0], len(pillars[i])
	pillarcount += len(pillars[i])
	rms += stat[i][0] * len(pillars[i])
	ellipse1 += stat[i][1] * len(pillars[i])
	ellipse2 += stat[i][2] * len(pillars[i])


numberPillars = sum([len(pillar) for pillar in pillars])/float(len(pillars))

print  '{}'.format("Average: "), '{}{}'.format("# ", numberPillars),  '{}{}'.format("rms: ", rms/float(pillarcount)), '{}{}'.format("ellipse: ", [ellipse1/float(pillarcount), ellipse2/float(pillarcount)])
# print   '{}'.format(numberPillars),  '{}{}'.format(",", numberPillars), '{}{}{}{}'.format(",", meanStat[1],",",meanStat[2])
csvf.writerow(["Average", numberPillars, rms/float(pillarcount), ellipse1/float(pillarcount), ellipse2/float(pillarcount)])


a = []
b = []
if showPath:
	print "hahahhahah"
	print file_path + file2 + "-path.txt"
	f = open(file_path + file2 + "-path.txt", 'r')
	for line in f:
		c = line.split(',')
		a.append(float(c[0]))
		b.append(-float(c[1]))
		print line

plt.plot(a,b,color=tableau20[14])

print "rms mean: ", rms/pillarcount
# plt.title(file)
ax.set_xlabel('x(m)')
ax.set_ylabel('y(m)')
ax.set_xlim(-5, 23)
ax.set_ylim(-2, 4)
ax.set(aspect=1, adjustable='box-forced')
# plt.savefig(file_path + file2 + '.png', bbox_inches='tight', dpi = 250)
# plt.show()



if showGrid:
	def f(x, A, B): # this is your 'straight line' y=f(x)
	    return A*x + B

	xl = []
	yl = []
	xh = []
	yh = []

	# Edit to make sure xl and yl are filled with the bottom row of pillars and that xh and yh with upper row
	for i in range(0,len(meanPillars)):
		if i==1 or i==3 or i==5 or i==8:
			xl.append(meanPillars[i][0])
			yl.append(meanPillars[i][1])
		if i==0 or i==4 or i==7:
			xh.append(meanPillars[i][0])
			yh.append(meanPillars[i][1])


	# Draw grid through the pillars
	# Lower pillars
	A,B = np.polyfit(xl, yl, 1)
	plt.plot([-5,23],[f(-5,A,B), f(23,A,B)], '--', color=tableau20[14], zorder=1000)

	# Higer pillars
	dist = 0
	for i in range(0,len(xh)):
		dist += distToLine(A,-1,B,xh[i],yh[i])
	dist /= 3.0
	# print "dist:", dist
	dist = dist/math.cos(math.atan2(A,1))
	plt.plot([-5,23],[f(-5,A,B+dist), f(23,A,B+dist)], '--', color=tableau20[14], zorder=1000)


	spacings = []
	# horizontal lines, find best spacing
	for i in range(0,len(xl)-1):
		spacings.append(math.sqrt((xl[i+1]-xl[i])**2 + (yl[i+1]-yl[i])**2))

	spacings = sum(spacings)/float(len(spacings))
	spacings = spacings/math.cos(math.atan2(-A,1))
	B = xl[0]+A*yl[0]
	plt.plot([f(-2,-A,B), f(4,-A,B)], [-2,4], '--', color=tableau20[14], zorder=1000)
	B = B+spacings
	plt.plot([f(-2,-A,B), f(4,-A,B)], [-2,4], '--', color=tableau20[14], zorder=1000)
	B = B+spacings
	plt.plot([f(-2,-A,B), f(4,-A,B)], [-2,4], '--', color=tableau20[14], zorder=1000)
	B = B+spacings
	plt.plot([f(-2,-A,B), f(4,-A,B)], [-2,4], '--', color=tableau20[14], zorder=1000)

	print xl[0], yl[0]

	print "grid size:", spacings, dist


# fig = figure()
# ax = fig.add_subplot(111, aspect='equal')
# ax.set_xlabel('x(m)')
# ax.set_ylabel('y(m)')
# ax.set_xlim(-5, 23)
# ax.set_ylim(-2, 4)
# for i in range(0,len(meanPillars)):
# 	alph = i/10.0+0.05
	# if i==1 or i==3 or i==5 or i==8:
		# plt.scatter(meanPillars[i][0], meanPillars[i][1],color='b')
plt.show()