import sys
import pylab

filename = sys.argv[1]
data = pylab.loadtxt(filename)
pylab.plot(data[:,0], data[:,1], '.')
pylab.axis('equal')
pylab.show()
