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
from srsim_result import SensorPlot

# single robot
class Robot:
    # === Init params configured from outside
    sim_dt = None # simulation dtime
    switch_sensor_plot = None # whether to plot sensor reading

    # === Params input from outside
    odor_sampling = None # odor sampling routine

    # === Params for exchanging data internally
    sim_step = None # for simulation step counting

    # === Params output
    # robot position
    robot_pos = None

    def __init__(self):
        # clear sim step count
        self.sim_step = 0
        # default plot sensor reading
        self.switch_sensor_plot = True

    def robot_init(self):
        # clear sim step count
        self.sim_step = 0
        # set robot position
        # plot sensor reading
        if self.switch_sensor_plot:
            self.plot_odor_sensor = SensorPlot('odor')
            #self.plot_odor_sensor.start_plot()

    def robot_update(self):
        # count sim step
        self.sim_step += 1
        # odor concentration value sampling
        odor_conc = self.odor_sampling(self.robot_pos)
        # plot sensor reading
        '''
        if self.switch_sensor_plot and self.plot_odor_sensor:
            self.plot_odor_sensor.sample[:] = \
                    [self.sim_step*self.sim_dt, odor_conc]
        '''

# multiple robots
class Robots:
    # robots' positions
    robot_pos = None
