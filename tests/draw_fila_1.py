"""
.. versionadded:: 1.1.0
   This demo depends on new features added to contourf3d.
"""

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm
import scipy.io as sio
import numpy as np
import math

fig = plt.figure()
ax = fig.gca(projection='3d')
path = u'../../vf_record_2016-01-26_172150/vortex_fila_564.mat'
fila = sio.loadmat(path)['vortex_fila']
# plot vortex fila
plot_fila = []
for i in range(fila.shape[1]): # rotors
    for j in range(fila.shape[2]): # blades
        x = fila[:,i,j,0]
        y = fila[:,i,j,1]
        z = fila[:,i,j,2]
        d = ax.plot3D(x,y,z)[0]
        plot_fila.append(d)

# draw blade
N_m = fila.shape[0]
x = np.array([fila[N_m-1,0,0,0], fila[N_m-1,0,1,0]])
y = np.array([fila[N_m-1,0,0,1], fila[N_m-1,0,1,1]])
z = np.array([fila[N_m-1,0,0,2], fila[N_m-1,0,1,2]])
blade = ax.plot3D(x,y,z, color='0.5')[0]
blade.set_linewidth(5)

# calculate vel
def biot_savart(a,b,p,Gamma):
    # AP, BP and AB vector
    ap = p - a
    bp = p - b
    ab = b - a
    # cos \theta_1 and cos \theta_2
    cos_theta_1 = np.dot(ab, ap)/(np.linalg.norm(ab)*np.linalg.norm(ap))
    cos_theta_2 = np.dot(-1*ab, bp)/(np.linalg.norm(ab)*np.linalg.norm(bp))
    # h, perpendicular distance from P to AB
    h = np.linalg.norm(ap) * math.sqrt(1 - math.pow(cos_theta_1, 2))
    # e, unit vector indicating the dir of induced velocity
    e = np.cross(ap, bp)
    e = e/np.linalg.norm(e)
    # induced velocity of this segment
    return Gamma/(4*np.pi)*(h/math.sqrt(0.01**4+h**4))*(cos_theta_1+cos_theta_2)*e

#  get the xyz lim of data fila
fila_xyzlim = np.zeros((3,2))
fila_xyzlim[2,0] = 3
for i in range(fila.shape[0]):
    for j in range(fila.shape[1]):
        for k in range(fila.shape[2]):
            for m in range(3): # xyz
                if fila[i,j,k,m] > fila_xyzlim[m,1]:
                    fila_xyzlim[m,1] = fila[i,j,k,m]
                elif fila[i,j,k,m] < fila_xyzlim[m,0]:
                    fila_xyzlim[m,0] = fila[i,j,k,m]
'''
#  obtain iso-surface vectors
x, y, z = np.mgrid[fila_xyzlim[0,0]-0.1:fila_xyzlim[0,1]+0.1:0.05,\
        0:1:1,\
        fila_xyzlim[2,0]-0.1:fila_xyzlim[2,1]+0.1:0.05]
u = np.zeros_like(x)
v = np.zeros_like(y)
w = np.zeros_like(z)
for i in range(x.shape[0]):
    for j in range(x.shape[1]):
        for k in range(x.shape[2]):
            p = np.array([x[i,j,k],y[i,j,k],z[i,j,k]])
            ind_v = np.zeros(3)
            for l in range(fila.shape[0]-1): # fila
                for m in range(fila.shape[1]): # rotor
                    for n in range(fila.shape[2]): # blade
                        if m == 0 or m == 2: # counter clockwise
                            a = fila[l+1,m,n]
                            b = fila[l,m,n]
                        else:
                            a = fila[l,m,n]
                            b = fila[l+1,m,n]
                        ind_v += biot_savart(a,b,p,0.1)
            u[i,j,k],v[i,j,k],w[i,j,k] = ind_v
#  draw
ax.quiver(x,y,z,u,v,w,length=0.018)
'''

ax.set_xlabel('X')
ax.set_xlim(fila_xyzlim[0,0]-0.15, fila_xyzlim[0,1]+0.15)
ax.set_ylabel('Y')
ax.set_ylim(fila_xyzlim[1,0]-0.15, fila_xyzlim[1,1]+0.15)
ax.set_zlabel('Z')
ax.set_zlim(fila_xyzlim[2,0]-0.15, fila_xyzlim[2,1]+0.15)

plt.show()
