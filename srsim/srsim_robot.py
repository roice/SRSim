#!/usr/bin/python
# coding=utf-8
#
# sniffer robots simulator
#                        Robot control
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

'''srsim robot module

Documentation and tests are included in ...
'''

# single robot
class Robot:
    # === Params input from outside
    odor_sampling = None # odor sampling routine

    # === Params for exchanging data internally

    # === Params output
    # robot position
    robot_pos = None

    #def robot_init(self):
        # set robot position

    def robot_update(self):
        # odor concentration value sampling
        odor_conc = self.odor_sampling(self.robot_pos)

# multiple robots
class Robots:
    # robots' positions
    robot_pos = None
