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
import fvmlib
import os, sys # for abs path & directory
import time

fig = plt.figure()
ax = fig.gca(projection='3d')
path = u'../../vf_record_2016-01-26_192714/vortex_fila_168.mat'
fila = sio.loadmat(path)['vortex_fila']

path = sys.path[0] # dir of this script
record_dir = path[0:path.index('SRSim/tests')] + 'draw_record_' + \
                time.strftime('%Y-%m-%d_%H%M%S',time.localtime(time.time()))
print('dir='+str(record_dir))
os.mkdir(record_dir)


'''
# plot vortex fila
plot_fila = []
for i in range(fila.shape[1]): # rotors
    for j in range(fila.shape[2]): # blades
        x = fila[:,i,j,0]
        y = fila[:,i,j,1]
        z = fila[:,i,j,2]
        d = ax.plot3D(x,y,z)[0]
        plot_fila.append(d)
'''

# draw blade
N_m = fila.shape[0]
for i in range(fila.shape[1]): # rotors
    x = np.array([fila[N_m-1,i,0,0], fila[N_m-1,i,1,0]])
    y = np.array([fila[N_m-1,i,0,1], fila[N_m-1,i,1,1]])
    z = np.array([fila[N_m-1,i,0,2], fila[N_m-1,i,1,2]])
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

#  obtain the velocity at a point
def get_velocity(p):
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
    return ind_v

# source is located at (-0.4, 0, 3.2)
pos_source = np.array([-0.4, 0, 3.2])
num_puffs = 50
pos_puffs = []
delta_t = 0.05
dir_psi = np.array([1.,-1.,1.,-1.])
for index_puff in range(num_puffs):
    temp_pos = np.copy(pos_source)
    for step in range(index_puff+1):
        temp_pos += fvmlib.get_vel_of_point(\
                fila, dir_psi,\
                temp_pos, delta_t, 0.2)*delta_t
    pos_puffs.append(temp_pos)

size_puffs = np.linspace(1,20,num_puffs)

pos_source_2 = np.array([0.3, 0.3, 3.2])
num_puffs_2 = 50
pos_puffs_2 = []
delta_t = 0.05
dir_psi = np.array([1.,-1.,1.,-1.])
for index_puff in range(num_puffs_2):
    temp_pos = np.copy(pos_source_2)
    for step in range(index_puff+1):
        temp_pos += fvmlib.get_vel_of_point(\
                fila, dir_psi,\
                temp_pos, delta_t, 0.2)*delta_t
    pos_puffs_2.append(temp_pos)

size_puffs = np.linspace(1,20,num_puffs)

sio.savemat(record_dir + '/pos_puffs_1.mat', \
                {'pos_puffs_1': pos_puffs})
sio.savemat(record_dir + '/pos_puffs_2.mat', \
                {'pos_puffs_2': pos_puffs_2})

#print('puffs='+str(pos_puffs))

# draw
x = np.array(pos_puffs)[:,0]
y = np.array(pos_puffs)[:,1]
z = np.array(pos_puffs)[:,2]
ax.scatter3D(x,y,z,s=size_puffs)
x = np.array(pos_puffs_2)[:,0]
y = np.array(pos_puffs_2)[:,1]
z = np.array(pos_puffs_2)[:,2]
ax.scatter3D(x,y,z,s=size_puffs)


ax.set_xlabel('X')
ax.set_xlim(fila_xyzlim[0,0]-0.15, fila_xyzlim[0,1]+0.15)
ax.set_ylabel('Y')
ax.set_ylim(fila_xyzlim[1,0]-0.15, fila_xyzlim[1,1]+0.15)
ax.set_zlabel('Z')
ax.set_zlim(fila_xyzlim[2,0]-0.15, fila_xyzlim[2,1]+0.15)

plt.show()
