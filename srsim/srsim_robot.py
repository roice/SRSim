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

import multiprocessing
import Queue

# single robot
class Robot:
    # === Init params configured from outside
    sim_dt = None # simulation dtime
    # communication process objects
    objs_comm_process = None

    # === Params input from outside
    odor_sampling = None # odor sampling routine, input from plume module

    # === Params for exchanging data internally
    sim_step = None # for simulation step counting

    # === Params output
    # robot position
    robot_pos = None

    def __init__(self):
        # clear sim step count
        self.sim_step = 0

    def robot_init(self):
        # clear sim step count
        self.sim_step = 0
        # set robot position
        # plot sensor reading

    def robot_update(self):
        # count sim step
        self.sim_step += 1
        # odor concentration value sampling
        odor_conc = self.odor_sampling(self.robot_pos)
        try:
            # send to communication process via queue
            self.objs_comm_process['odor_sample'].put(\
                [self.sim_step*self.sim_dt, odor_conc], block=True, timeout=5)
            waypoint = self.objs_comm_process['robot_waypoint'].get(block=True, timeout=5)
        except Queue.Full:
            if self.objs_comm_process['sim_state'][2] == -1: # client stopped
                self.flush_queues()
            else:
                print 'SRsim robot: odor sample queue is full, is algorithm platform (client) running?'
        except Queue.Empty:
            if self.objs_comm_process['sim_state'][2] == -1: # client stopped
                self.flush_queues()
            else:
                print 'SRsim robot: robot waypoint queue is empty, is algorithm platform (client) running?'

    def flush_queues(self):
    # flush contents of queues of communication
        if not self.objs_comm_process['odor_sample'].empty():
            tmp = self.objs_comm_process['odor_sample'].get()
        if not self.objs_comm_process['robot_waypoint'].empty():
            tmp = self.objs_comm_process['robot_waypoint'].get()

# multiple robots
class Robots:
    # robots' positions
    robot_pos = None
