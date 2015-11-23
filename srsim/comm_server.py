#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# sniffer robots simulator
#                        Communication server
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
Communicate with approach
'''

import socket
import Queue # for exception handler
import time

class CommServer:
    # === Params configured from outside
    queue_odor_sample = None
    queue_robot_waypoint = None
    shared_sim_state = None
    mode = None
    address = None
    port = None

    def start(self):
        # check if right data channels have been configured
        if self.queue_odor_sample is None or self.queue_robot_waypoint is None\
                or self.shared_sim_state is None:
            exit('Comm server: data channel have not been configured yet!')
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # determine the address and port
        if self.mode == 'local':
            sock.bind(('localhost', 60000))
            print "SRsim server is listenting port 6000 of address: " + 'localhost'
        elif self.mode == 'distributed':
            sock.bind((self.address, self.port))
            print "SRsim server is listenting port " + self.port + " of address: " + self.address
        else:
            exit('Comm server: Do you run srsim & algorithm on a same computer or not?')
        sock.listen(1) # permit only one connection
        # tell main process the address&port sucessfully bind
        self.shared_sim_state[1] = 1
        while True:
            # wait for client
            connection,address = sock.accept()
            # client is connecting, tell main process
            self.shared_sim_state[2] = 1
            try:
                # establish connection, set time out
                connection.settimeout(5) # timeout sec
                # receive request from client
                buf = connection.recv(1024)
                # see what the client requests
                #  '>': from srsim to client
                if buf == '$SRsim>odor_sample':
                    # client requests odor concentration value
                    #  check if simulation is running
                    if self.shared_sim_state[0] == 0: # if sim's not running
                        # tell client the simulation ended
                        connection.send('$SRsim=end=')
                    else:
                        #  get odor sample value
                        odor_sample = self.queue_odor_sample.get(\
                                    block=True, timeout=3) # timeout sec
                        #  send odor value
                        connection.send(str(odor_sample))
                        #  get robot waypoint
                        str_recv = connection.recv(1024)
                        #  check if it's valid data
                        if str_recv.find('[') >= 0 and str_recv.find(']') >= 0:
                            self.queue_robot_waypoint.put(eval(str_recv), \
                                    block=True, timeout=3)
                        else:
                            # seems connection failed, or client closed
                            self.shared_sim_state[2] = -1 # tell main process that client stopped
                            self.flush_queues()
                elif buf == '$SRsim?start?': # if sim started
                        # client is asking whether the sim is started or not
                        #  send start/end state of sim to the client
                        if self.shared_sim_state[0] == 0: # if sim's not running
                            connection.send('$SRsim=end=')
                        else:
                            connection.send('$SRsim=start=')
                connection.close()
            except socket.timeout:
                print 'SRsim server connection time out, seems client broke connection'
                if self.shared_sim_state[0] == 1: # if sim's running
                    if self.shared_sim_state[2] == 1: # if main process thought the client's linking
                        self.shared_sim_state[2] = -1 # tell main process that client stopped
                    elif self.shared_sim_state[2] == -2: # probably there's no client while sim's running
                        self.shared_sim_state[2] = -2 # tell main process that there's no client
                else: # if sim's not running
                    if self.shared_sim_state[2] != -2:
                        self.shared_sim_state[2] = -2
                self.flush_queues()
            except Queue.Full:
                if self.shared_sim_state[0] == 1: # if sim's running
                    print "SRsim sim doesn't receive robot waypoint from server, it's weird"
                self.flush_queues()
            except Queue.Empty:
                if self.shared_sim_state[0] == 1: # if sim's running
                    print "SRsim server can't get odor sample, it's weird"
                self.flush_queues()

    def flush_queues(self):
        # flush queues
        if not self.queue_odor_sample.empty():
            tmp = self.queue_odor_sample.get()
        if not self.queue_robot_waypoint.empty():
            tmp = self.queue_robot_waypoint.get()

##############################################################################
# Execute if running this script
if __name__ == '__main__':
    comm = CommServer()
    comm.start()
