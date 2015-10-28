#!/usr/bin/python
# coding=utf-8
#
# sniffer robots simulator
#                       loop
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

'''srsim loop
Contains main loop of srsim
Documentation and tests are included in ...
'''

from threading import Thread
from time import sleep
from srsim_wind_model import srsim_wind_simulator
#from srsim_plume_model import srsim_plume_simulator
from pyface.api import GUI

class SimulationThread(Thread):
    ''' Simulation loop.
    '''
    # flag indicating whether to abort sim
    wants_abort = False

    def run(self):
        ''' Runs the simulation loop
        '''

        # Reset simulation step
        sim_step = 0
        while not self.wants_abort:
            sim_step += 1
            ''' Update wind field '''
            ''' Update pollutant diffusion '''
            ''' Update robot position '''
            # retrieve mesh grid
            x, y, z = self.grid
            # compute wind field
            if (self.wind_model != 'const'):# prevent useless computing
                self.data_wind_field = srsim_wind_simulator(self.wind_model, self.grid, [1,2,3], 0)
            # compute odor field
            #srsim_plume_simulator('farrell', self.grid, self.data_wind_field, self.odor_source_pos, self.sim_step)
            # update sim count & sim scene visualization
            self.count_sim_step()
            GUI.invoke_later(self.update_scene)
            sleep(0.02)
        self.display('###### Simulation stopped ######')
