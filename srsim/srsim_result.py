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
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# for exception
from sys import exit
# for debug
from time import sleep
import multiprocessing

# sensor reading plot
class SensorPlot:
    # font type
    label_font = {'family': 'serif',
        'color':  'black',
        'weight': 'normal',
        'size': 16,
        }
    # === Params input from outside
    # sample data, refreshed every sample interval
    #   type: multiprocessing shared state
    #         <SynchronizedArray wrapper for
    #           <multiprocessing.sharedctypes.c_float_Array_2 object...
    #   content: [time, value], self.sample[:] == [time, value]
    # how to get value?
    # right way 1: self.sample[0] or self.sample[1]
    # right way 2: self.sample[:]
    # wrong way:   X = self.sample, or self.sample = X..list
    sample = None

    # === Params for internal data exchanging
    data = None # a list
    len_recent_display = 10 # display recent 100 samples on the 1st subplot

    def __init__(self, sensor_type):
        # check if valid sensor type is choosen
        if sensor_type != 'odor':
            exit('Error: Sendor type "' + str(sensor_type) + \
                    '" is not supported')
        # save sensor type for displaying different labels
        self.sensor = sensor_type
        self.data = [[0., 0.]] # 0 s, 0 molecues

    # this generator must exist and must generate data every interval,
    #  it's 'update' function's job to judge whether to plot it
    def sampling(self):
        while True:
            sample = self.sample
            yield sample

    # displays dynamically a certain period of reading
    def init(self, shared_mem):
        self.sample = shared_mem
        # 2 subplot
        fig, self.ax = plt.subplots(2)
        # 2 subplot
        #   1st subplot: plot a certain period of reading
        #   2nd subplot: plot all reading from the epoch
        self.plot_1, = self.ax[0].plot(self.data[0], color='g', linewidth=2)
        self.plot_2, = self.ax[1].plot(self.data[0], color='b', linewidth=2)
        # display label/title according to sensor type
        if self.sensor == 'odor': # an odor sensor
            self.ax[0].set_title('Recent odor sensor reading', \
                    fontdict=self.label_font)
            self.ax[1].set_title('Odor sensor reading from epoch', \
                    fontdict=self.label_font)
            self.ax[1].set_xlabel('time (s)', fontdict=self.label_font)
            plt.ylabel('Concentration ('+r'$\frac{molecules}{cm^3}$'+')', \
                    fontdict=self.label_font)
        # Tweak spacing to prevent clipping of ylabel
        plt.subplots_adjust(left=0.15)
        ani = animation.FuncAnimation(fig, self.update, self.sampling,\
                blit=False, interval=10, repeat=False)
        plt.show()

    # update plots
    # reading is a list containing 2 float, [time, concentration]
    def update(self, reading):
        # check if this is a new data
        #   reading: sample: [time, value]
        #   if the time value in this reading is not equal to the latest time
        #   value in data list, then it should be a new data
        if reading and reading[0] != self.data[len(self.data)-1][0]:
            # update reading list
            self.data.append([reading[0], reading[1]])
            # update the data & axis limit of 1st subplot
            N = len(self.data)
            if N > self.len_recent_display: # recent data reading
                self.plot_1.set_xdata(np.array(self.data)\
                        [N-self.len_recent_display:N,0])
                self.plot_1.set_ydata(np.array(self.data)\
                        [N-self.len_recent_display:N,1])
                self.ax[0].set_xlim(float(min(np.array(self.data)\
                        [N-self.len_recent_display:N,0])), \
                        float(max(np.array(self.data)\
                        [N-self.len_recent_display:N,0])))
                self.ax[0].set_ylim(float(min(np.array(self.data)\
                        [N-self.len_recent_display:N,1])), \
                        float(max(np.array(self.data)\
                        [N-self.len_recent_display:N,1])))
            else:
                self.plot_1.set_xdata(np.array(self.data)[:,0])
                self.plot_1.set_ydata(np.array(self.data)[:,1])
                self.ax[0].set_xlim(float(min(np.array(self.data)[:,0])), \
                    float(max(np.array(self.data)[:,0])))
                self.ax[0].set_ylim(float(min(np.array(self.data)[:,1])), \
                    float(max(np.array(self.data)[:,1])))
            # update the data & axis limit of 1st subplot
            self.plot_2.set_xdata(np.array(self.data)[:,0])
            self.plot_2.set_ydata(np.array(self.data)[:,1])
            self.ax[1].set_xlim(float(min(np.array(self.data)[:,0])), \
                    float(max(np.array(self.data)[:,0])))
            self.ax[1].set_ylim(float(min(np.array(self.data)[:,1])), \
                    float(max(np.array(self.data)[:,1])))

    def start_plot(self):
        # Although it's not safe to use shared states, but there's no choice
        data = multiprocessing.Array('f', [0.,0.])
        # create child process for plotting
        self.plot_process = multiprocessing.Process(target=self.init,\
                args=([data]))
        # link data obj to self.sample in this (parent) instance,
        # Note: it's different from the sample attribute of child process's
        #  target self.init, while linking data to this self.sample made data
        #  obj being changeable by changing self.sample
        self.sample = data
        self.plot_process.start()

    def close_plot(self):
        print 'now close plot'
        self.plot_process.join()
        print 'closed'

# Result Plot function
#  input params:
#    sensor: a sensor type list, ['odor', ...]
def ResultPlot(sensor):
    for name in sensor:
        if name == 'odor': # plot odor sensor
            print 'plot odor sensor'
            p = SensorPlot('odor')
            p.start_plot()

##############################################################################
# Execute if running this script
if __name__ == '__main__':
    p = SensorPlot('odor')
    p.start_plot()
    a = [0.,0.]
    for i in range(100):
        a[0] += 0.1
        a[1] += 0.1
        p.sample[:] = a
        sleep(0.1)
    p.close_plot()
