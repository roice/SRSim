#!/usr/bin/python
# coding=utf-8
#
# sniffer robots simulator
#                        Wind models
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

'''srsim wind module

Documentation and tests are included in ...
'''

import numpy as np
from fipy import Grid3D, CellVariable, DiffusionTerm
from scipy.interpolate import griddata
#for debug
from mayavi import mlab

class Advection():
    # === Params configured from outside ===
    # wind model selection
    wind_model_sel = None
    # sim area size
    gsize = None  # grid size
    xyz_n = None  # x,y,z grid numbers, e.g., (100, 100, 100) for cubic sim area
                  # of edge length 100*gsize
    # mean wind vector
    mean_flow = None
    # colored noise process parameters
    wind_damping = None
    wind_bandwidth = None
    G = None
    # simulation period, dt (second)
    dt = None

    # === Parameters for exchanging data between functions ===
    mesh = None
    # wind vector random increment of vertex points, static variables
    vertexWindVecRanInc = None
    # wind mesh cell centers
    wind_mesh_centers = None
    # wind result /phi
    wind_phi_field = None

    # === Results ===
    wind_vector_field = None
    wind_at_vertex = None # wind vector at 8 vertexes, also the BCs, for interp

    # imcompressible, inviscid, irrotational, rectangular sim area
    # step 1 & step 2: Mesh & Equation
    def icivir_cuboid_init(self):
        # step 1: Mesh
        self.mesh = Grid3D(dx=self.gsize, dy=self.gsize, dz=self.gsize, \
                nx=self.xyz_n[0], ny=self.xyz_n[1], nz=self.xyz_n[2])
        # clear colored noise static parameters
        self.vertexWindVecRanInc = [ [ [0 for i in range(2)] for i in range(3)] for i in range(8)]

    # step 3 & step 4: BCs & Solve
    # positions of 8 vertexes:
    #
    #           vertex 2 ------------ vertex 3                y
    #             / |                   /|                    |
    #            /                     / |                    |
    #           /                     /  |                    |
    #       vertex 6 ------------ vertex 7                    /------x
    #          |    |                |   |                   /
    #          | vertex 0            | vertex 1             /
    #          |  /                  | /                   z
    #       vertex 4 ------------ vertex 5
    def icivir_cuboid_solve(self):
        # step 2: Equation
        phi = CellVariable(mesh=self.mesh, name='potential phi', value=0.)
        eqn = (DiffusionTerm(coeff = 1.) == 0.)
        # step 3: Boundary conditions
        # compute flow of 8 vertexes
        # one vertex has 3 components of wind vector, (x, y, z)
        vertexWindVec = np.array([ [0.0 for i in range(3)] for i in range(8)])
        for i in range(8): # 8 vertexes
            for j in range(3): # 3 components
                vertexWindVec[i, j] = self.mean_flow[j] \
                        + self.colored_noise(self.vertexWindVecRanInc[i][j])
        # save these 8 vector for interp wind of edge area for plume sim
        self.wind_at_vertex = vertexWindVec
        #print 'vertexWindVec = ' + str(vertexWindVec)
        # interpolate flow vector on sim area faces, and set neumann boundary
        #   conditions, because /grad /phi = V(x,y,z)
        # /grad /phi array, of points of mesh face centers
        #   grad_phi_bc[0, :]  /grad/phi_x
        #   grad_phi_bc[1, :]  /grad/phi_y
        #   grad_phi_bc[2, :]  /grad/phi_z
        grad_phi_bc = np.zeros_like(self.mesh.faceCenters())
        # p: points on one face to interpolate, 2 dimension
        # vx, vy: x&y components of interpolated wind vectors of points list p
        # vertex index, for interpolate bc on 6 faces
        # vertexIndex[0] = [0,1,4,5], down  face, 0<x<nx*dx, y=0, 0<z<nz*dz
        # vertexIndex[1] = [0,2,4,6], left  face, x=0, 0<y<ny*dy, 0<z<nz*dz
        # vertexIndex[2] = [2,3,6,7], up    face, 0<x<nx*dx, y=ny*dy, 0<z<nz*dz
        # vertexIndex[3] = [1,3,5,7], right face, x=nx*dx, 0<y<ny*dy, 0<z<nz*dz
        # vertexIndex[4] = [0,1,2,3], front face, 0<x<nx*dx, 0<y<ny*dy, z=0
        # vertexIndex[5] = [4,5,6,7], back  face, 0<x<nx*dx, 0<y<ny*dy, z=nz*dz
        # write vertexIndex as mask type
        vertexMask = np.array([ \
                [True, True, False, False, True, True, False, False],
                [True, False, True, False, True, False, True, False],
                [False, False, True, True, False, False, True, True],
                [False, True, False, True, False, True, False, True],
                [True, True, True, True, False, False, False, False],
                [False, False, False, False, True, True, True, True] ])
        xyz_index = np.array([ [0,2], [1,2], [0,2], [1,2], [0,1], [0,1] ])
        for i in range(6): # 6 faces for 3D cuboid area
            p1, p2 = np.mgrid[self.gsize/2:self.gsize*self.xyz_n[xyz_index[i,0]]:self.gsize, \
                    self.gsize/2:self.gsize*self.xyz_n[xyz_index[i,1]]:self.gsize]
            vx = griddata(zip(\
                    [0, self.gsize*self.xyz_n[xyz_index[i,0]], 0, self.gsize*self.xyz_n[xyz_index[i,0]]],\
                    [0, 0, self.gsize*self.xyz_n[xyz_index[i,1]], self.gsize*self.xyz_n[xyz_index[i,1]]]),\
                    vertexWindVec[vertexMask[i],0], (p1, p2), method='linear').T.reshape(1,-1)[0]
            vy = griddata(zip(\
                    [0, self.gsize*self.xyz_n[xyz_index[i,0]], 0, self.gsize*self.xyz_n[xyz_index[i,0]]],\
                    [0, 0, self.gsize*self.xyz_n[xyz_index[i,1]], self.gsize*self.xyz_n[xyz_index[i,1]]]),\
                    vertexWindVec[vertexMask[i],1], (p1, p2), method='linear').T.reshape(1,-1)[0]
            vz = griddata(zip(\
                    [0, self.gsize*self.xyz_n[xyz_index[i,0]], 0, self.gsize*self.xyz_n[xyz_index[i,0]]],\
                    [0, 0, self.gsize*self.xyz_n[xyz_index[i,1]], self.gsize*self.xyz_n[xyz_index[i,1]]]),\
                    vertexWindVec[vertexMask[i],2], (p1, p2), method='linear').T.reshape(1,-1)[0]
            if i == 0: # down boundary
                grad_phi_bc[:, self.mesh.facesDown()] = np.array([vx, vy, vz])
            elif i == 1: # left boundary
                grad_phi_bc[:, self.mesh.facesLeft()] = np.array([vx, vy, vz])
            elif i == 2: # up boundary
                grad_phi_bc[:, self.mesh.facesUp()] = np.array([vx, vy, vz])
            elif i == 3: # right boundary
                grad_phi_bc[:, self.mesh.facesRight()] = np.array([vx, vy, vz])
            elif i == 4: # front
                grad_phi_bc[:, self.mesh.facesFront()] = np.array([vx, vy, vz])
            elif i == 5: # back
                grad_phi_bc[:, self.mesh.facesBack()] = np.array([vx, vy, vz])
        #print 'grad_phi_bc[ext] = ' + str(grad_phi_bc[:, self.mesh.exteriorFaces()])
        # set neumann boundary condition
        phi.faceGrad.constrain(((grad_phi_bc[0]),(grad_phi_bc[1]), (grad_phi_bc[2])), where=self.mesh.exteriorFaces)
        # set dirichlet boundary condition
        # set /phi value on one point of a cell, to provide a init value for equaition solver
        #   get all points which lie on the center of faces of cells
        X, Y, Z = self.mesh.faceCenters
        mask = ((X == self.gsize/2) & (Y == self.gsize/2) & (Z == 0)) # front face of cell 0
        phi.constrain(0, where=self.mesh.exteriorFaces & mask)
        # step 4: Solve
        eqn.solve(var=phi)
        # Post processing
        #   get /phi array
        self.wind_phi_field = np.array(phi).reshape(self.xyz_n[2], \
                self.xyz_n[1], self.xyz_n[0]).T
        #   convert /phi to wind vector
        self.wind_vector_field = np.array(np.gradient(self.wind_phi_field, self.gsize))
        #print 'wind_vector_field = ' + str(self.wind_vector_field)
        #   get cell centers
        self.wind_mesh_centers = self.mesh.cellCenters()

    # init class attributes
    def __init__(self):
        self.gsize = 1.0    # float
        self.xyz_n = [3, 4, 5]
        self.mean_flow = [1.0, 0.0, 0.0]
        self.wind_damping = 0.3
        self.wind_bandwidth = 0.5
        self.G = 30
        self.dt = 0.1
        # simulation area's shape is cuboid, which has 8 vertexes
        # one vertex has 3 components of wind vector, (x, y, z)
        # one component of wind vector has 2 elements, e.g., x and x', y and y'
        self.vertexWindVecRanInc = [ [ [0 for i in range(2)] for i in range(3)] for i in range(8)]

    # colored noise process
    #
    # input: u(t), white noise, 'standard normal',
    #        unit variance normal distribution (mean 0 variance 1)
    # output: x(t), wind variance
    #
    # transfer function (G * classical second-order system):
    #
    #                       w_n^2
    # H(s) = G * ---------------------------
    #               s^2 + 2*z*w_n + w_n^2
    #                                       wind_bandwidth^2
    #      = wind_meander*------------------------------------------------------
    #                     s^2 + 2*wind_damping*wind_bandwidth + wind_bandwidth^2
    # Time domain:
    #   d^2                                        d
    #  -----x(t) + 2*wind_damping*wind_bandwidth*----x(t) + wind_bandwidth^2*x(t)
    #  dt^2                                       dt
    #      = G*wind_bandwidth^2*u(t)
    def colored_noise(self, x):
        # sample from standard normal
        u = np.random.randn()
        # init local params
        dx = [0, 0]
        #        d
        # eq 1: ----x(t) = x'
        #        dt
        dx[0] = x[1]
        #        d^2                                        d
        # eq 2: ----x(t) = -2*wind_damping*wind_bandwidth*----x(t)
        #       dt^2                                       dt
        #                 + wind_bandwidth^2*(G*u(t) - x(t))
        dx[1] = -2*self.wind_damping*self.wind_bandwidth*dx[0] \
                + self.wind_bandwidth*self.wind_bandwidth*(self.G*u-x[0])
        #                  d^2
        # eq 3: x' = /int -----x(t)
        #                 dx^2
        x[1] += dx[1] * self.dt
        # eq 4: x = /int x'
        x[0] += x[1] * self.dt
        return x[0]

    '''
    Constant flow:
    Uniform time-invariant wind model
    input: x/y/z    points to sample, mgrid arrays
    input: vector   uniform wind vector
    output: u/v/w   wind vector arrays
    '''
    def uniform_tinv(self):
        self.mesh = np.mgrid[self.gsize/2:self.gsize*self.xyz_n[0]:self.gsize, \
                self.gsize/2:self.gsize*self.xyz_n[1]:self.gsize,
                self.gsize/2:self.gsize*self.xyz_n[2]:self.gsize]
        # init u/v/w arrays as the same shape of x/y/z
        v = np.ones_like(self.mesh)
        # get u/v/w value
        self.wind_vector_field = np.array([v[i]*self.mean_flow[i] for i in range(3)])
        # save these 8 vector for interp wind of edge area for plume sim
        self.wind_at_vertex = np.array([ [self.mean_flow[i] for i in range(3)] for i in range(8)])

    # uniform time variant, wind = meanflow + stochastic_variance
    def uniform_stochastic(self):
        # compute variance
        var = [self.colored_noise(self.vertexWindVecRanInc[0][i]) for i in range(3)]
        # get wind vector
        wind = np.array(self.mean_flow) + np.array(var)
        # get u/v/w arrays
        v = np.ones_like(self.mesh)
        self.wind_vector_field = np.array([v[i]*wind[i] for i in range(3)])
        self.wind_at_vertex = np.array([ [wind[i] for i in range(3)] for i in range(8)])

    # call wind simulation function according to wind model selected
    '''
    input:
        wind_model_sel --- selected wind model name, type str
            'constant'      : constant wind field, time invariant
            'uniform'       : uniform wind field, time variant
            'irrotational'  : incompressible, inviscid, irrotational flow
            'ext'           : load external data
    '''
    def wind_init(self, sel):
        if (sel == 'constant'):
            #print 'Wind: uniform wind field selected'
            self.uniform_tinv() # compute only once
        elif (sel == 'uniform'):
            self.uniform_tinv() # use this function to init
            # clear colored noise static parameters
            self.vertexWindVecRanInc = [ [ [0 for i in range(2)] for i in range(3)] for i in range(8)]
        elif (sel == 'irrotational'):
            self.icivir_cuboid_init()
        self.wind_model_sel = sel
    def wind_update(self):
        if (self.wind_model_sel == 'uniform'):
            self.uniform_stochastic()
        elif (self.wind_model_sel == 'irrotational'):
            self.icivir_cuboid_solve()

##############################################################################
# Execute if running this script
if __name__ == '__main__':
    from timeit import Timer
    adv = Advection()
    adv.icivir_cuboid_init()
    t1=Timer("adv.icivir_cuboid_solve()")
    print 't1 = ' + str(t1.timeit(10))
    #adv.icivir_cuboid_solve()
    '''
    # display
    x, y, z = np.mgrid[adv.gsize/2.0:adv.gsize*adv.xyz_n[0]:adv.gsize, \
            adv.gsize/2.0:adv.gsize*adv.xyz_n[1]:adv.gsize, \
            adv.gsize/2.0:adv.gsize*adv.xyz_n[2]:adv.gsize,]
    u, v, w = adv.wind_vector_field
    #print 'x = ' + str(x)
    #print 'u.shape = ' + str(u.shape)
    mlab.quiver3d(x,y,z,u,v,w)
    # axes and outlines
    mlab.axes( xlabel = 'X East (m)', ylabel = 'Y North (m)', \
            zlabel = 'Z Up (m)', ranges = [0, adv.gsize*adv.xyz_n[0], 0, \
            adv.gsize*adv.xyz_n[1], 0, adv.gsize*adv.xyz_n[2]])
    mlab.outline(extent=[0, adv.gsize*adv.xyz_n[0], 0, \
            adv.gsize*adv.xyz_n[1], 0, adv.gsize*adv.xyz_n[2]],)
    mlab.show()
    '''
