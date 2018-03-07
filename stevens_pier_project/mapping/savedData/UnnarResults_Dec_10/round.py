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

def num(s):
    try:
        return int(s)
    except ValueError:
    	pass
    try:
        return round(float(s),2)
    except ValueError:
    	return s
        # return int(s)


csv_out2 = open('pillar-average.csv', 'w')
csvf2 = csv.writer(csv_out2)

for i in range(0,6):

	# print("hmmmm")
	# file_path = os.path.join(parent_directory, 'savedData/')
	csv_out = open('pillar0'+str(i)+'-new-rounded.csv', 'w')
	csvf = csv.writer(csv_out)


	with open('pillar0'+str(i)+'-new.csv', 'r') as csvfile:
		spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
		for row in spamreader:
			a=[]
			for col in row:
				a.append(num(col))
				# print(num(col))
			csvf.writerow(a)
	a[0] = i/10.0
	print a
	csvf2.writerow(a)