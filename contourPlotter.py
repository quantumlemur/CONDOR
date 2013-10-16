import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

fileName = 'Output/designSpace_thinkpad-ubuntu_1.csv'


gw = []
hoverceiling = []
maxrange = []
maxspeed = []
payload = []
cost = []
with open(fileName, 'rb') as f:
    reader = csv.DictReader(f)
    keys = sorted(reader.fieldnames)
    for row in reader:
        gw.append(row['GrossWeight'])
        hoverceiling.append(row['HoverCeiling'])
        maxrange.append(row['MaxRange'])
        maxspeed.append(row['MaxSpeed'])
        payload.append(row['Payload'])
        cost.append(row['HarrisScullyPrice'])

r = np.linspace(300, 1000, 5)
s = np.linspace(100, 250, 5)
g = griddata((maxspeed, maxrange), gw, (s[None,:], r[:,None]), method='cubic')
CS = plt.contourf(s, r, g)
plt.colorbar()
plt.show()


# import numpy as np
# from scipy.interpolate import griddata
# import matplotlib.pyplot as plt
# import numpy.ma as ma
# from numpy.random import uniform, seed
# # make up some randomly distributed data
# seed(1234)
# npts = 200
# x = uniform(-2,2,npts)
# y = uniform(-2,2,npts)
# z = x*np.exp(-x**2-y**2)
# # define grid.
# xi = np.linspace(-2.1,2.1,100)
# yi = np.linspace(-2.1,2.1,100)
# # grid the data.
# zi = griddata((x, y), z, (xi[None,:], yi[:,None]), method='cubic')
# # contour the gridded data, plotting dots at the randomly spaced data points.
# CS = plt.contour(xi,yi,zi,15,linewidths=0.5,colors='k')
# CS = plt.contourf(xi,yi,zi,15,cmap=plt.cm.jet)
# plt.colorbar() # draw colorbar
# # plot data points.
# plt.scatter(x,y,marker='o',c='b',s=5)
# plt.xlim(-2,2)
# plt.ylim(-2,2)
# plt.title('griddata test (%d points)' % npts)
# plt.show()