#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# sniffer robots simulator
#                        Communication client
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

'''
Communicate with srsim
'''

import socket
import time
# for exception
from sys import exit
import errno # for catching socket 'connectin refused' error
# for test
import multiprocessing

class CommClient:
    # Params configured form outside
    odor_sample = None
    robot_waypoint = None

    def start(self):
        # check if right data channels have been configured
        if self.odor_sample is None or self.robot_waypoint is None:
            exit('Comm client: data channel have not been configured yet!')
        while True:
            try:
                # create connection
                connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                connection.connect(('localhost', 60000))
                # connection established, request for odor value
                connection.send('$SRsim>odor_sample')
                str_recv = connection.recv(1024)
                # check if data received is valid
                if str_recv == '$SRsim=end=':
                # simulation is ending (or not start), wait for starting
                    print 'SRsim simulation not running'
                    # tell algorithm platform & result plotting that simulation is not running
                    if self.sim_state[0] == 1:
                            self.sim_state[0] = 0
                    time.sleep(1)
                else:
                    # check if it's valid data
                    if str_recv.find('[') >= 0 and str_recv.find(']') >= 0:
                        self.odor_sample[:] = eval(str_recv) # receive odor sample
                        connection.send(str(self.robot_waypoint[:])) # send robot waypoint
                        # tell algorithm platform & result plotting that simulation started
                        if self.sim_state[0] == 0:
                            self.sim_state[0] = 1
                connection.close()
            except socket.error, v:
                errorcode = v[0]
                if errorcode == errno.ECONNREFUSED:
                    print "Connection to SRsim sever Refused"
                    # tell algorithm platform & result plotting that simulation is not running
                    if self.sim_state[0] == 1:
                            self.sim_state[0] = 0
                    time.sleep(1) # wait 1 sec

##############################################################################
# Execute if running this script
if __name__ == '__main__':
    # create shared states for data transfering
    # Although it's not safe to use shared states, but there's no choice
    odor_sample = multiprocessing.Array('f', [0.,0.])
    robot_waypoint = multiprocessing.Array('f', [0.,0.,0.])
    comm = CommClient()
    comm.odor_sample = odor_sample
    comm.robot_waypoint = robot_waypoint
    comm.start()
