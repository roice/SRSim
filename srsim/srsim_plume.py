#!/usr/bin/python
# coding=utf-8
#
# sniffer robots simulator
#                ---- plume model
#
# Author: Roice Luo <oroice@foxmail.com>
# copyright (c) 2015 Roice Luo <https://github.com/roice>
#
# This library is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published by the
# Free Software Foundation; either version 2.1 of the License, or (at your
# option) any later version.
#
# This library is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License
# for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this library; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

'''srsim plume model module

Documentation and tests are included in ...
'''

import numpy as np
from scipy import interpolate
# for debug
from mayavi import mlab
from sys import exit

# Farrell's filament-based atmospheric dispersion model
# Notations:
# this file                 in Farrell's PlumeOdor MFC class
# fila[:]['x']              puff[i].xx
# fila[:]['y']              puff[i].yy
# fila[:]['z']              puff[i].zz
# fila[:]['r']              puff[i].R^3
class FilamentModel:
    # === Input params ===
    # odor source position
    odor_source_pos = None
    # sim area size
    sim_area_size = None
    # sim time interval
    dt = None
    # instantaneous advective wind vector field
    adv_field = None # mesh value
    adv_mesh = None # mesh
    adv_xyz_n = None # mesh grid numbers in 3 dims [nx,ny,nz]
    adv_gsize = None # mesh grid size, float
    adv_vertex = None # wind vector at 8 vertexes
    # /sigma_v for centerline dispersion, v_m
    vm_sigma = None
    # fila growth rate
    fila_growth_rate = None
    # fila number per second
    fila_number_per_sec = None
    odor_release_rate = None # release rate, molecules/s

    # === Internal exchanging data ===
    # advection mesh & vector field of the whole area
    va_mesh = None
    va_field = None
    # the filament number should release
    fila_number_need_release = None
    # molecules per filament
    mole_per_filament = None
    # data structure type of odor filament
    fila_type = np.dtype({ \
            'names':['x','y','z','r'], \
            'formats':['f','f','f','f']})
    # odor filament list
    fila = None

    def plume_init(self):
        # release the first odor pack at source pos
        self.fila = np.array([], dtype = self.fila_type)
        self.odor_release()
        # clear
        self.fila_number_need_release = 0.0
        # whole area advection mesh
        #  +0.1: in order to include sim_area_size[i]
        self.va_mesh = np.mgrid[ \
                0:self.sim_area_size[0]+0.1:1.0, 0:self.sim_area_size[1]+0.1:1.0, \
                0:self.sim_area_size[2]+0.1:1.0]

    def plume_update(self):
        # Step 1: integrate positions and sizes of fila
        # centerline relative dispersion
        vm = [ [self.vm_sigma*np.random.randn() for i in range(3)] for i in range(len(self.fila)) ]
        # get flow vectors at fila positions
        #   interp whole area, as advection field is calculated only at mesh cell centers
        #     coordinate of 8 vertex
        vertex = np.array([ \
                [0, 0, 0],
                [self.adv_gsize*self.adv_xyz_n[0], 0, 0],
                [0, self.adv_gsize*self.adv_xyz_n[1], 0],
                [self.adv_gsize*self.adv_xyz_n[0], self.adv_gsize*self.adv_xyz_n[1], 0],
                [0, 0, self.adv_gsize*self.adv_xyz_n[2]],
                [self.adv_gsize*self.adv_xyz_n[0], 0, self.adv_gsize*self.adv_xyz_n[2]],
                [0, self.adv_gsize*self.adv_xyz_n[1], self.adv_gsize*self.adv_xyz_n[2]],
                [self.adv_gsize*self.adv_xyz_n[0], self.adv_gsize*self.adv_xyz_n[1], \
                        self.adv_gsize*self.adv_xyz_n[2]]])
        #    interpolate wind advection on finer grids, including the edge area
        va = [[],[],[]] # wind advection on finer grid
        #    get known points for quick cycle
        points = np.array(zip( \
                    np.hstack((self.adv_mesh[0].reshape(1,-1)[0], vertex[:,0])), \
                    np.hstack((self.adv_mesh[1].reshape(1,-1)[0], vertex[:,1])), \
                    np.hstack((self.adv_mesh[2].reshape(1,-1)[0], vertex[:,2]))))
        for i in range(3): # 3 dim
            va[i] = np.array(interpolate.griddata(points, \
                    np.hstack((self.adv_field[i].reshape(1,-1)[0], self.adv_vertex[:,i])), \
                    (self.va_mesh[0], self.va_mesh[1], self.va_mesh[2]), method = 'linear'))
        #  interpolate wind advection at fila's positions
        f_x = interpolate.RegularGridInterpolator((self.va_mesh[0,:,0,0], \
                self.va_mesh[1,0,:,0], self.va_mesh[2,0,0,:]), va[0])
        f_y = interpolate.RegularGridInterpolator((self.va_mesh[0,:,0,0], \
                self.va_mesh[1,0,:,0], self.va_mesh[2,0,0,:]), va[1])
        f_z = interpolate.RegularGridInterpolator((self.va_mesh[0,:,0,0], \
                self.va_mesh[1,0,:,0], self.va_mesh[2,0,0,:]), va[2])
        va_fila = np.array(zip( \
                f_x(np.array(zip(self.fila[:]['x'], self.fila[:]['y'], self.fila[:]['z']))), \
                f_y(np.array(zip(self.fila[:]['x'], self.fila[:]['y'], self.fila[:]['z']))), \
                f_z(np.array(zip(self.fila[:]['x'], self.fila[:]['y'], self.fila[:]['z'])))))
        # Euler integration of fila position
        delta_pos = (va_fila + vm)*self.dt
        self.fila['x'] += delta_pos[:,0]
        self.fila['y'] += delta_pos[:,1]
        self.fila['z'] += delta_pos[:,2]
        # fila grow
        self.fila['r'] += np.ones_like(self.fila['r'])*self.fila_growth_rate*self.dt
        # eliminate fila which are out of sim area
        mask = np.array([])
        for i in range(len(self.fila)):
            if (self.fila[i]['x'] < 0) or (self.fila[i]['y'] < 0) or (self.fila[i]['z'] < 0) \
                    or (self.fila[i]['x'] > self.sim_area_size[0]) \
                    or (self.fila[i]['y'] > self.sim_area_size[1]) \
                    or (self.fila[i]['z'] > self.sim_area_size[2]):
                mask = np.append(mask, i)
        self.fila = np.delete(self.fila, mask, 0)
        # count number of fila which should be released, and release odor
        self.fila_number_need_release += self.dt*self.fila_number_per_sec
        if self.fila_number_need_release >= 1.0:
            for i in range(int(self.fila_number_need_release)):
                self.odor_release()
            self.fila_number_need_release -= int(self.fila_number_need_release)

    def odor_release(self):
        new_fila = np.array([(self.odor_source_pos[0], self.odor_source_pos[1],\
                self.odor_source_pos[2], 0.001)], dtype = self.fila_type)
        self.fila = np.append(self.fila, new_fila)

    # sample the odor concentration value at robot's position
    # the concentration at location x due to the i-th filament (location p_i) is modeled as
    #                       Q
    #  C_i(x) = --------------------------- exp(- delta' * SIGMA^(-1) * delta)
    #            sqr(8*pi^3 * det(SIGMA))
    #  delta = x - p_i
    #
    #            sigma
    #  SIGMA = [      sigma     ] when wind vector at p_i is (0,0,0)
    #                      sigma
    def odor_conc_value_sampling(self, pos):
        # use filament model as default plume model
        #   get fila number
        N = len(self.fila)
        #   calculate SIGMA of all fila
        SIGMA = np.array([ [[1, 0, 0],[0, 1, 0],[0, 0, 1]] for i in range(N) ])
        #   calculate det(SIGMA)
        SIGMA_det = np.linalg.det(SIGMA)
        #   calculate SIGMA^(-1)
        SIGMA_inv = np.linalg.inv(SIGMA)
        #   calculate delta of robot pos to all fila
        delta = np.array([ [[\
                    pos[0] - self.fila[i]['x'], \
                    pos[1] - self.fila[i]['y'], \
                    pos[2] - self.fila[i]['z']]] for i in range(N) ])
        #   calculate delta'
        delta_T = np.array([ delta[i].T for i in range(N) ])
        # calculate exp power
        power = np.array([ -np.dot(np.dot(delta[i], SIGMA_inv[i]),delta_T[i])[0,0] for i in range(N) ])
        # concentration contribution due to all fila
        conc_fila = self.mole_per_filament/np.power(8*np.power(np.pi, 3)*SIGMA_det, 1./2.)*np.exp(power)
        # concentration at this position
        conc = np.dot(np.array([conc_fila]), np.array([[1.] for i in range(N)]))[0,0]
        return conc

    def __init__(self):
        self.vm_sigma = 2.0 # m/s/sqr(Hz), float
        self.fila_growth_rate = 0.001 # expand 0.001 meter per second
        self.fila_number_per_sec = 10 # release 10 fila per sec, integer
        self.odor_release_rate = 10.0 # molecules/s, must be float
        self.mole_per_filament = self.odor_release_rate/self.fila_number_per_sec

# plume model wrapper
#  example: plume = Dispersion('farrell')
class Dispersion:
    def __init__(self, model):
        if model == 'farrell':
            self.model = FilamentModel()
        else:
            exit('Error: Other plume model not supported yet...')

###############################################################
# Execute if running this script
if __name__ == '__main__':
    plume = FilamentModel()
    plume.adv_gsize = 1.0
    plume.adv_xyz_n = [3, 4, 5]
    plume.adv_mesh = np.mgrid[ \
            plume.adv_gsize/2.0:plume.adv_gsize*plume.adv_xyz_n[0]:plume.adv_gsize, \
            plume.adv_gsize/2.0:plume.adv_gsize*plume.adv_xyz_n[1]:plume.adv_gsize, \
            plume.adv_gsize/2.0:plume.adv_gsize*plume.adv_xyz_n[2]:plume.adv_gsize]
    plume.adv_field = np.zeros_like(plume.adv_mesh)
    for i in range(plume.adv_xyz_n[0]):
        for j in range(plume.adv_xyz_n[1]):
            for k in range(plume.adv_xyz_n[2]):
                plume.adv_field[0,i,j,k] = 1.0 # (1.0, 0, 0) m/s
    plume.adv_vertex = np.array([ [1.0, 0, 0] for i in range(8) ])
    plume.vm_sigma = 2.0
    plume.sim_area_size = [3, 4, 5]
    plume.odor_source_pos = [plume.sim_area_size[0]/4.0, plume.sim_area_size[1]/2.0, plume.sim_area_size[2]/2.0]
    plume.dt = 0.1
    plume.fila_number_per_sec = 10
    # init plume
    plume.plume_init()

    while(True):
        plume.plume_update()
        print 'length of fila = ' + str(len(plume.fila))
