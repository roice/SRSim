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
import multiprocessing
import numpy as np
# SRsim
from srsim_wind import Advection
from srsim_plume import FilamentModel
from srsim_robot import Robot

class SimulationThread(Thread):
    ''' Simulation loop.
    '''
    # === Params input from outside
    # flag indicating whether to abort sim, controled by outside
    wants_abort = False

    # === Params output to outside
    # data for outside display
    wind_vector_field = None
    plume_snapshot = None

    def SimulationProcess(self, sim_step, queue_wind_data, queue_odor_data,\
            sim_dt, sim_area_size, odor_source_pos, \
            wind_gsize, wind_mesh, wind_model, wind_mean_flow, \
            wind_G, wind_damping, wind_bandwidth, \
            plume_vm_sigma, plume_fila_growth_rate, plume_fila_number_per_sec, \
            robot_init_pos, \
            objs_comm_process):
        # create & init wind field calculation instance
        wind = Advection()
        l, w, h = np.array(wind_mesh).shape[1:4] # 1 2 3
        wind.xyz_n = [l, w, h]
        wind.gsize = wind_gsize
        wind.mean_flow = list(wind_mean_flow)
        wind.G = wind_G
        wind.wind_damping = wind_damping
        wind.wind_bandwidth = wind_bandwidth
        wind.wind_init(wind_model)
        # create & init plume field computation instance
        plume = FilamentModel()
        plume.odor_source_pos = odor_source_pos
        plume.sim_area_size = sim_area_size
        plume.dt = sim_dt
        plume.adv_mesh = wind_mesh
        plume.adv_xyz_n = [l, w, h]
        plume.adv_gsize = wind_gsize
        plume.vm_sigma = plume_vm_sigma
        plume.fila_growth_rate = plume_fila_growth_rate
        plume.fila_number_per_sec = plume_fila_number_per_sec
        plume.plume_init()
        # create & init robot control instance
        robot = Robot()
        robot.robot_pos = robot_init_pos
        robot.sim_dt = sim_dt
        robot.odor_sampling = plume.odor_conc_value_sampling
        robot.objs_comm_process = objs_comm_process
        robot.robot_init()
        # Reset simulation step
        sim_step.value = 0
        while True:
            # Update wind field
            wind.wind_update()
            # Update pollutant diffusion
            plume.adv_field = wind.wind_vector_field
            plume.adv_vertex = wind.wind_at_vertex
            plume.plume_update()
            # Update robot position
            robot.robot_update()
            sim_step.value += 1
            # send wind result to sim thread for display in mayavi scene
            queue_wind_data.put(wind.wind_vector_field, block=True)
            # send odor result to sim thread for display in mayavi scene
            queue_odor_data.put(plume.fila, block=True)

    def run(self):
        ''' Run the simulation loop, the computation is running in a child process
        '''
        # --- Create Simulation Compute Process
        # create shared states for data transfering
        # Although it's not safe to use shared states, but there's no choice
        #  count simulation step
        sim_step_count = multiprocessing.Value('i', 0)
        # queue for passing wind vector field data
        queue_wind_data = multiprocessing.Queue(maxsize=1)
        # queue for passing odor fila list
        queue_odor_data = multiprocessing.Queue(maxsize=1)
        # create simulation process
        self.sim_process = multiprocessing.Process(target=self.SimulationProcess, \
                args=(sim_step_count, queue_wind_data, queue_odor_data, \
                self.sim_dt, self.sim_area_size, self.odor_source_pos, \
                self.wind_gsize, self.wind_mesh, self.wind_model, self.wind_mean_flow, \
                self.wind_G, self.wind_damping, self.wind_bandwidth, \
                self.plume_vm_sigma, self.plume_fila_growth_rate, self.plume_fila_number_per_sec, \
                self.robot_init_pos, \
                self.objs_comm_process))
        # start simulation process
        self.sim_process.start()

        while not self.wants_abort:
            self.wind_vector_field = queue_wind_data.get(block=True)
            self.plume_snapshot = queue_odor_data.get(block=True)
            # check if client is running
            if self.objs_comm_process['sim_state'][2] >= 0: # client normal
                # sim scene visualization
                GUI.invoke_later(self.update_scene)
            else: # client is stopped/in trouble, stop simulation
                self.wants_abort = True
                GUI.invoke_later(self.stop_simulation)

        # terminate simulation process
        self.sim_process.terminate()
        self.sim_process.join()
        # flush contents of queues created in this thread
        if not queue_wind_data.empty():
            tmp = queue_wind_data.get()
        if not queue_odor_data.empty():
            tmp = queue_odor_data.get()
        # flush contents of queues of communication
        if not self.objs_comm_process['odor_sample'].empty():
            tmp = self.objs_comm_process['odor_sample'].get()
        if not self.objs_comm_process['robot_waypoint'].empty():
            tmp = self.objs_comm_process['robot_waypoint'].get()
        # print
        self.add_text_line('###### Simulation stopped ######')
