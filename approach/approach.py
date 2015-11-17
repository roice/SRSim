#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# sniffer robots simulator
#                        Result plotting module
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

'''srsim result plotting module
   plot sensor reading, algorithm results...

Documentation and tests are included in ...
'''

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# SRsim
from comm_client import CommClient
# for exception
from sys import exit
# for debug
from time import sleep
import multiprocessing

# Result plotting, the instance of this class will enter a infinite loop after initialize
#  And, it should run in the main thread/process
class ResultPlot:
    # font type
    label_font = {'family': 'serif',
        'color':  'black',
        'weight': 'normal',
        'size': 16,
        }
    # === Params input from outside
    # sample data of odor, refreshed every sample interval
    #   type: multiprocessing shared state
    #         <SynchronizedArray wrapper for
    #           <multiprocessing.sharedctypes.c_float_Array_2 object...
    #   content: [time, value], self.odor_sample[:] == [time, value]
    # how to get value?
    # right way 1: self.odor_sample[0] or self.odor_sample[1]
    # right way 2: self.odor_sample[:]
    # wrong way:   X = self.odor_sample, or self.odor_sample = X..list
    odor_sample = None
    # odor statistical map ...

    # === Params for internal data exchanging
    odor_time_series = None # odor sample list, time series
    len_recent_sample_disp = 10 # number of recent samples which will display on the sensor reading subplot
    # plot switch
    switch_odor_reading_plot = False # plot odor reading if True
    # plot clean flag, the fig is clean if True
    plot_clean = True
    # process to close when close event trigged (when pressed close button of the figures)
    process_to_close_at_close_event = None

    def __init__(self, command):
        # check validation of plot command, 'command' is a string list, e.g., ['odor', '...']
        for order in command:
            # check if valid sensor type is chosen
            if order == 'odor':
                self.switch_odor_reading_plot = True
            else:
                exit('Error: plot command "' + str(order) + \
                    '" is not supported')
        # data init
        self.odor_time_series = [[0., 0.]] # 0 s, 0 molecues/cm^3

    # this generator must exist and must generate data every interval,
    #  it's 'update' function's job to judge whether to plot it
    def plot_data_generator(self):
        while True:
            if self.switch_odor_reading_plot:
                # plot odor reading
                odor_sample = self.odor_sample[:]
            else:
                odor_sample = None
            yield {'odor_sample': odor_sample}

    # displays dynamically a certain period of reading
    def init(self):
        # check if data pipes have been established
        #  the data pipes should be created after create instance of this class,
        #   and before calling this func
        if self.switch_odor_reading_plot:
            if self.odor_sample is None:
                exit('Error: odor_sample is None while odor reading plot is switched on')
            else:
                # plot odor reading, init
                (fig, self.ax_odor_ts, self.plot_odor_ts1, self.plot_odor_ts2) = \
                        self.odor_reading_plot_init()
        # Tweak spacing to prevent clipping of ylabel
        plt.subplots_adjust(left=0.15)
        ani = animation.FuncAnimation(fig, self.update, self.plot_data_generator,\
                blit=False, interval=10, repeat=False)
        # set close handle
        fig.canvas.mpl_connect('close_event', self.handle_close)
        # GUI loop
        plt.show()

    # update plots
    def update(self, plot_data):
        if self.switch_odor_reading_plot:
            # check if sim stopped/not running, need to init odor time series
            if self.sim_state[0] == 0:
                # data init
                self.odor_time_series = [[0., 0.]] # 0 s, 0 molecues/cm^3
            else: # sim's running
                # reading is a list containing 2 float, [time, concentration]
                odor_sample = plot_data['odor_sample']
                # check if this is a new data
                #   reading: sample: [time, value]
                #   if the time value in this reading is not equal to the latest time
                #   value in data list, then it should be a new data
                if odor_sample and odor_sample[0] is not self.odor_time_series[len(self.odor_time_series)-1][0]:
                    # if this is a new fig (if len(odor_time_series) == 1), need clean it first (usually after re-start)
                    if self.plot_clean == False: # plot's dirty
                        # clean figure
                        self.odor_reading_plot_update()
                        self.plot_clean = True
                    # update reading list
                    self.odor_time_series.append(odor_sample)
                    self.odor_reading_plot_update()
                    # tell the next iteration that this figure is dirty
                    if self.plot_clean == True:
                        self.plot_clean = False

    def handle_close(self, evt):
        print('Approach figures closed, bye~')
        # close communication process
        if self.process_to_close_at_close_event:
            self.process_to_close_at_close_event.terminate()
            self.process_to_close_at_close_event.join()

    # === Private plotting functions
    # reset odor reading plot function
    def odor_reading_plot_init(self):
        # 2 subplot
        fig, ax_odor_ts = plt.subplots(2)
        # 2 subplot
        #   1st subplot: plot a certain period of reading
        #   2nd subplot: plot all reading from the epoch
        plot_1, = ax_odor_ts[0].plot(self.odor_time_series[0], color='g', linewidth=2)
        plot_2, = ax_odor_ts[1].plot(self.odor_time_series[0], color='b', linewidth=2)
        # display label/title according to sensor type
        ax_odor_ts[0].set_title('Recent odor sensor reading', \
                fontdict=self.label_font)
        ax_odor_ts[1].set_title('Odor sensor reading from epoch', \
                fontdict=self.label_font)
        ax_odor_ts[1].set_xlabel('time (s)', fontdict=self.label_font)
        ax_odor_ts[1].set_ylabel('Concentration ('+r'$\frac{molecules}{cm^3}$'+')', \
                fontdict=self.label_font)
        return (fig, ax_odor_ts, plot_1, plot_2)
    def odor_reading_plot_update(self):
        # update the data & axis limit of 1st subplot
        N = len(self.odor_time_series)
        if N > self.len_recent_sample_disp: # recent data reading
            # update data
            self.plot_odor_ts1.set_xdata(np.array(self.odor_time_series)\
                    [N-self.len_recent_sample_disp:N,0])
            self.plot_odor_ts1.set_ydata(np.array(self.odor_time_series)\
                    [N-self.len_recent_sample_disp:N,1])
            # update axis
            self.ax_odor_ts[0].set_xlim(float(min(np.array(self.odor_time_series)\
                    [N-self.len_recent_sample_disp:N,0])) - 1.0, \
                    float(max(np.array(self.odor_time_series)\
                    [N-self.len_recent_sample_disp:N,0])) + 1.0)
            self.ax_odor_ts[0].set_ylim(float(min(np.array(self.odor_time_series)\
                    [N-self.len_recent_sample_disp:N,1])) - 1.0, \
                    float(max(np.array(self.odor_time_series)\
                    [N-self.len_recent_sample_disp:N,1])) + 1.0)
        else:
            # update data
            self.plot_odor_ts1.set_xdata(np.array(self.odor_time_series)[:,0])
            self.plot_odor_ts1.set_ydata(np.array(self.odor_time_series)[:,1])
            # update axis
            self.ax_odor_ts[0].set_xlim(float(min(np.array(self.odor_time_series)[:,0])) - 1.0, \
                    float(max(np.array(self.odor_time_series)[:,0])) + 1.0)
            self.ax_odor_ts[0].set_ylim(float(min(np.array(self.odor_time_series)[:,1])) -1.0, \
                    float(max(np.array(self.odor_time_series)[:,1])) + 1.0)
        # update the data & axis limit of 2nd subplot
        self.plot_odor_ts2.set_xdata(np.array(self.odor_time_series)[:,0])
        self.plot_odor_ts2.set_ydata(np.array(self.odor_time_series)[:,1])
        self.ax_odor_ts[1].set_xlim(float(min(np.array(self.odor_time_series)[:,0])) -1.0, \
                float(max(np.array(self.odor_time_series)[:,0])) + 1.0)
        self.ax_odor_ts[1].set_ylim(float(min(np.array(self.odor_time_series)[:,1])) -1.0, \
                float(max(np.array(self.odor_time_series)[:,1])) + 1.0)

def communication(odor_sample, robot_waypoint, sim_state):
    comm = CommClient()
    comm.odor_sample = odor_sample
    comm.robot_waypoint = robot_waypoint
    comm.sim_state = sim_state
    comm.start()

##############################################################################
# Execute if running this script
if __name__ == '__main__':
    # -- create communication process
    # create shared states for data transfering
    # Although it's not safe to use shared states, but there's no choice
    odor_sample = multiprocessing.Array('f', [0.,0.])
    robot_waypoint = multiprocessing.Array('f', [0.,0.,0.])
    # simulation state indicator
    # [start/end, ...]
    # start/end: simulation is running when start/end == 1, ended when start/end == 0
    shared_sim_state = multiprocessing.Array('i', [0, 0])

    # create child process for communicaiton with srsim
    #  this process retrieves odor concentration sample from srsim, and give
    #   robot's next waypoint to srsim for robot dynamic path control(/planning)
    process_comm_with_srsim = multiprocessing.Process(target=communication,\
                args=(odor_sample, robot_waypoint, shared_sim_state))
    process_comm_with_srsim.start()

    # -- plotting
    plot = ResultPlot(['odor'])
    plot.odor_sample = odor_sample
    plot.sim_state = shared_sim_state
    plot.process_to_close_at_close_event = process_comm_with_srsim
    plot.init()
