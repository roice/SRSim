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

# this file             in farrell's PlumeOdor MFC class
# puff[:]['pos_x']:         puff[i].xx
# puff[:]['pos_y']:         puff[i].yy
# puff[:]['pos_z']:         puff[i].zz
# puff[:]['radius']:        puff[i].R^3
pufftype = np.dtype({
    'names':['pos_x','pos_y','pos_z','radius']
    'formats':['f','f','f','f']})
puff = np.array([(0,0,0,0)])

def srsim_plume_farrell(grid, current_wind_field, source_pos, sim_step):
    # get parameters
    x_mesh, y_mesh, z_mesh = grid #wind field cartesian coord grid mesh
    # convert 3d mesh grid to 3 1D array (x,y,z) form, for the convenience
    #                                                   of interpolating
    x = x_mesh[:, 0, 0]
    y = y_mesh[0, :, 0]
    z = z_mesh[0, 0, :]
    u, v, w = current_wind_field  #wind field data
    sp_x, sp_y, sp_z = source_pos #source cartesian coord position

    # init when this function first called
    if sim_step == 1
        puff = np.array([(sp_x, sp_y, sp_z, 0.001)])

    # centerline relative dispersion, random driving noise

    interpolate.interpn((x,y,z), (u,v,w), (puff[:]['pos_x'],puff[:]['pos_y'],
        puff[:]['pos_z']), method='linear', bounds_error=True, fill_value=nan)
    # Euler integration of puff position
    puff
    return o

def srsim_plume_simulator(plume_model_sel, wind_grid, current_wind_field, source_pos, sim_step):
    if (plume_model_sel == 'farrell'):
        return srsim_plume_farrell(wind_grid, current_wind_field, source_pos, sim_step)
