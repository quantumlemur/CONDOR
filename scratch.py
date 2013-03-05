

import numpy as np
from bisect import bisect_left


c81File='Config/NACA 63-015.c81'
airfoildata = np.genfromtxt(c81File, skip_header=0, skip_footer=0) # read in the airfoil file

# print airfoildata
# print airfoildata.shape


airfoildata = [map(float, line.strip().split('\t')) for line in open(c81File)]
#print airfoildata



def interpolate(xout, xin, yin, method='linear'):
    """
    Interpolate the curve defined by (xin, yin) at points xout. The array
    xin must be monotonically increasing. The output has the same data type as
    the input yin.

    :param yin: y values of input curve
    :param xin: x values of input curve
    :param xout: x values of output interpolated curve
    :param method: interpolation method ('linear' | 'nearest')

    @:rtype: numpy array with interpolated curve
    """
    lenxin = len(xin)

    i1 = np.array([bisect_left(xin, xo) for xo in xout])
    #i1[ i1==0 ] = 1
    #i1[ i1==lenxin ] = lenxin-1

    x0 = xin[i1-1]
    x1 = xin[i1]
    y0 = yin[i1-1]
    y1 = yin[i1]

    if method == 'linear':
        return (xout - x0) / (x1 - x0) * (y1 - y0) + y0
    elif method == 'nearest':
        return np.where(np.abs(xout - x0) < np.abs(xout - x1), y0, y1)
    else:
        raise ValueError('Invalid interpolation method: %s' % method)# Python code here



a = np.arange(10).astype(np.float)
b = np.arange(10).astype(np.float) * 5
print a
print b
print interpolate([0.5], a, b)[0]