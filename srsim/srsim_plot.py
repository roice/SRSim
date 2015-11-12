#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# sniffer robots simulator
#                        Plotting module
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

'''srsim plotting module
   plot sensor reading, algorithm results...

Documentation and tests are included in ...
'''

import numpy as np
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
    #   sample: [time, value]
    sample = None

    # === Params for internal data exchanging
    data = None # a list

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
            print 'sample = ' + str(self.sample)
            sample = self.sample
            yield sample

    # displays dynamically a certain period of reading
    def init(self, arr):
        self.sample = arr
        # 1 subplot
        fig, self.ax = plt.subplots()
        # subplot, plot a certain period of reading
        self.plot, = self.ax.plot(self.data[0])
        if self.sensor == 'odor': # an odor sensor
            self.ax.set_title('Odor sensor reading', fontdict=self.label_font)
            self.ax.set_xlabel('time (s)', fontdict=self.label_font)
            self.ax.set_ylabel('Concentration ()', fontdict=self.label_font)
        ani = animation.FuncAnimation(fig, self.update, self.sampling,\
                blit=False, interval=100, repeat=False)
        plt.show()

    # update plots
    # reading is a list containing 2 float, [time, concentration]
    def update(self, reading):
        #print 'reading = ' + str(reading)
        # check if this is a new data
        #   reading: sample: [time, value]
        #   if the time value in this reading is not equal to the latest time
        #   value in data list, then it should be a new data
        if reading and reading[0] != self.data[len(self.data)-1][0]:
            # update reading list
            self.data.append([reading[0], reading[1]])
            if len(self.data) > 100: # 100 reading
                del self.data[0]
            # change axes limits of plots
            self.ax.set_xlim(float(min(self.data[:][0])), \
                    float(max(self.data[:][0])))
            self.ax.set_ylim(float(min(self.data[:][0])), \
                    float(max(self.data[:][0])))
            # change data streamline
            self.plot.set_xdata(self.data[:][0])
            self.plot.set_ydata(self.data[:][0])

#plt.text(2, 0.65, r'$\cos(2 \pi t) \exp(-t)$', fontdict=font)

# Tweak spacing to prevent clipping of ylabel
#plt.subplots_adjust(left=0.15)


##############################################################################
# Execute if running this script
if __name__ == '__main__':
    def data_gen():
        for i in range(100):
            data[0] += 0.1
            data[1] += 0.1
            #plot.sample = data
            sleep(0.1)
    plot = SensorPlot('odor')
    data = multiprocessing.Array('f', [0,0])
    print 'data = '+str(data)
    # create a child process to do plotting job
    plot_process = multiprocessing.Process(target=plot.init, args=(data))
    plot_process.start()

    print 'generate data'
    data_gen()
