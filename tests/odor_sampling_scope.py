"""
Odor sensor sampling value dynamically plot
"""
import time
import numpy as np
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt

# sensor reading plot
class SensorPlot:
    # font
    label_font = {'family': 'serif',
        'color':  'black',
        'weight': 'normal',
        'size': 16,
        }

    def __init__(self, sensor_type):
        # save sensor type for displaying different labels
        self.sensor = sensor_type

    # 2 subplot, one displays dynamically a certain period of reading,
    #  the other displays all reading from the epoch
    def init(self):
        plt.ion() # turn interactive mode on
        # 2 subplot => 2 data sets, y[0] and y[1] lists
        self.x = [ [0.], [0.] ]
        self.y = [ [0.], [0.] ]
        # 1 figure, 2 subplots
        f, self.axarr = plt.subplots(2)
        # 1st subplot, plot a certain period of reading
        self.plot_1, = self.axarr[0].plot(self.x[0], self.y[0])
        self.axarr[0].set_title('Odor sensor reading', fontdict=self.label_font)
        self.axarr[0].set_xlabel('time (s)', fontdict=self.label_font)
        self.axarr[0].set_ylabel('Concentration ()', fontdict=self.label_font)
        # 2nd subplot, plot all reading from the epoch
        self.plot_2, = self.axarr[1].plot(self.x[1], self.y[1])
        # show plots
        plt.show()

    # update plots
    # reading is a list containing 2 float, [time, concentration]
    def update(self, reading):
        # update reading list
        self.x[0].append(reading[0])
        self.x[1].append(reading[0])
        self.y[0].append(reading[1])
        self.y[1].append(reading[1])
        if len(self.x[0]) > 100: # 100 reading
            del self.x[0][0]
            del self.y[0][0]
        # change axes limits of plots
        self.axarr[0].set_xlim(float(min(self.x[0])), float(max(self.x[0])))
        self.axarr[1].set_xlim(float(min(self.x[1])), float(max(self.x[1])))
        self.axarr[0].set_ylim(float(min(self.y[0])), float(max(self.y[0])))
        self.axarr[1].set_ylim(float(min(self.y[1])), float(max(self.y[1])))
        # change data streamline
        self.plot_1.set_xdata(self.x[0])
        self.plot_1.set_ydata(self.y[0])
        self.plot_2.set_xdata(self.x[1])
        self.plot_2.set_ydata(self.y[1])
        # update plots
        #self.axarr[0].draw()
        #self.axarr[1].draw()
        plt.draw()

#plt.text(2, 0.65, r'$\cos(2 \pi t) \exp(-t)$', fontdict=font)

# Tweak spacing to prevent clipping of ylabel
#plt.subplots_adjust(left=0.15)


##############################################################################
# Execute if running this script
if __name__ == '__main__':
    plot = SensorPlot('odor')
    plot.init()
    data = [0,0]
    while True:
        data[0] += 0.1
        data[1] += 0.1
        plot.update(data)
