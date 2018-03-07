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

# print "tot: ", (226+141+86)/float(226+141+86+3+37+7+1+33+3)


# pnpre = 252/float(252+4+10)
# pnrec = 252/float(252+8+34)
# pnacc = (252+140+3+1+84)/float(252+140+3+1+84 + 4+10+8+34)

# pwpre = 140/float(140 + 8+3)
# pwrec = 140/float(140+4+1)
# pwacc = (140+252+10+85+35)/float((140+252+10+85+35)+4+8+3+1)

# pppre = 85/float(85 + 34+1)
# pprec = 85/float(85+10+3)
# ppacc = (85+252+4+8+140)/float((85+252+4+8+140)+34+1+10+3)

# print pnpre
# print pnrec
# print pnacc
# print pwpre
# print pwrec
# print pwacc
# print pppre
# print pprec
# print ppacc
# print "tot: ", (252+140+85)/float(252+140+85+4+10+3+8+34+1)


# rnpre = 253/float(253+4+9)
# rnrec = 253/float(253+3+35)
# rnacc = (253+144+4+1+84)/float(253+144+4+1+84 + 4+9+3+35)

# rwpre = 144/float(144 + 3+4)
# rwrec = 144/float(144+4+1)
# rwacc = (144+253+9+84+35)/float((144+253+9+84+35)+4+3+1+4)

# rppre = 84/float(84 + 35+1)
# rprec = 84/float(84+9+4)
# rpacc = (84+253+4+3+144)/float((84+253+4+3+144)+9+4+35+1)

# print rnpre
# print rnrec
# print rnacc
# print rwpre
# print rwrec
# print rwacc
# print rppre
# print rprec
# print rpacc

print "tot: ", (253+144+84)/float(253+144+84+4+9+4+1+35+3)
