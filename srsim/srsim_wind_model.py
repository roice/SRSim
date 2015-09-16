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

'''srsim wind model module

Documentation and tests are included in ...
'''

import numpy as np

''' Uniform time-invariant wind model
input: x/y/z    points to sample, mgrid arrays
input: vector   uniform wind vector
output: u/v/w   wind vector arrays
'''
def srsim_wind_uniform_tinv_get_uvw(x, y, z, vector):
    # init u/v/w arrays as the same shape of x/y/z
    u = np.ones_like(x)
    v = np.ones_like(y)
    w = np.ones_like(z)
    # get u/v/w valui/e
    u = vector[0]*u
    v = vector[1]*v
    w = vector[2]*w
    return u, v, w
