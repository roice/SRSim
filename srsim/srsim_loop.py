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
from pyface.api import GUI
from srsim_result import ResultPlot
import multiprocessing

class SimulationThread(Thread):
    ''' Simulation loop.
    '''
    # === Init params configured from outside
    sim_dt = None # simulation dtime
    wind_model = None # wind model selection
    count_sim_step = None # sim step count function in srsim_ui.py
    update_scene = None # update scene signal, srsim_ui.py
    display = None # print text to outside item in srsim_ui.py
    wind = None # instance of wind field calculation class
    plume = None # instance of plume calculation class
    robot = None # instance of robot navigation class

    # === Params input from outside
    # flag indicating whether to abort sim
    wants_abort = False

    # === Params output to outside
    # data for outside display
    wind_vector_field = None
    plume_snapshot = None

    def run(self):
        ''' Runs the simulation loop
        '''
        # Reset simulation step
        sim_step = 0
        # Init simulation
        #  Init wind field
        self.wind.wind_init(self.wind_model)
        #  Init plume field
        self.plume.plume_init()
        #  Init robot control
        self.robot.sim_dt = self.sim_dt # for sensor plot
        self.robot.robot_init()
        #  Init result plotting
        self.result_process = multiprocessing.Process(target=ResultPlot, \
                args=([['odor','fuckyou']]))
        self.result_process.start()
        while not self.wants_abort:
            sim_step += 1
            # Update wind field
            self.wind.wind_update()
            #    copy wind result to local for outside display
            self.wind_vector_field = self.wind.wind_vector_field
            # Update pollutant diffusion
            self.plume.adv_field = self.wind_vector_field
            self.plume.adv_vertex = self.wind.wind_at_vertex
            self.plume.plume_update()
            #    copy plume result to local for outside display
            self.plume_snapshot = self.plume.fila
            # Update robot position
            self.robot.robot_update()
            # update sim count & sim scene visualization
            self.count_sim_step()
            GUI.invoke_later(self.update_scene)
            #sleep(0.005)
        self.display('###### Simulation stopped ######')
