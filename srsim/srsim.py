#!/usr/bin/python
# coding=utf-8
#
# sniffer robots simulator
#                        Main
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

'''srsim

Documentation and tests are included in ...
'''

import multiprocessing
import Queue # for exception handle
import time
# SRsim
from srsim_ui import MainWindow
from comm_server import CommServer
import srsim_config

def SRsimUI(communication_p, q_odor_sample, q_robot_waypoint, sh_sim_state):
    mainwin = MainWindow()
    mainwin.comm_process = communication_p
    mainwin.queue_odor_sample = q_odor_sample
    mainwin.queue_robot_waypoint = q_robot_waypoint
    mainwin.shared_sim_state = sh_sim_state
    mainwin.configure_traits()

def communication(q_odor_value, q_robot_waypoint, sh_sim_state, mode, address, port):
    # create communication server
    comm = CommServer()
    comm.mode = mode
    comm.address = address
    comm.port = port
    comm.queue_odor_sample = q_odor_value
    comm.queue_robot_waypoint = q_robot_waypoint
    comm.shared_sim_state = sh_sim_state
    comm.start()

# create queue for data transfer
queue_odor_sample = multiprocessing.Queue(maxsize=1) # output odor samples
queue_robot_waypoint = multiprocessing.Queue(maxsize=1) # input robot waypoints
# create shared states for data transfering
#  Although it's not safe to use shared states, but there's no choice
# simulation state indicator
# [start/end, comm_state, client_state, ...]
# start/end: simulation is running when start/end == 1, ended when start/end == 0
# comm_state: communication state
#             if value is -1:   address/port bind failed
#             if value is 1:    normal state
# client_state: client state
#             if value is 0:    no client
#             if value is 1:    normal, linking
#             if value is -1:   seems suddenly stopped
shared_sim_state = multiprocessing.Array('i', [0, -1, 0])

# get address & port number from config
srsim_config.load_settings()
# create communication process
comm_process = multiprocessing.Process(target=communication, args=(\
        queue_odor_sample, queue_robot_waypoint, shared_sim_state, \
        srsim_config.get_comm_mode(), srsim_config.get_comm_address(), srsim_config.get_comm_port()))
comm_process.start()
# transfer process handle and queues to SRsim GUI
SRsimUI(comm_process, queue_odor_sample, queue_robot_waypoint, shared_sim_state)
