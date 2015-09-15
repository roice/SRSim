#!/usr/bin/python
# coding=utf-8
#
# sniffer robots simulator user interface
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

'''srsim ui module

Documentation and tests are included in ...
'''

# Major scientific library imports
import numpy as np
# Enthought imports
from traits.api import HasTraits, Array, Instance, Button, Enum, String,\
        Range, Array, Tuple, Float
from traitsui.api import View, Item, Group, HSplit, VSplit,\
        ButtonEditor, EnumEditor, RangeEditor
from mayavi.core.ui.api import MlabSceneModel, SceneEditor
# Thread related imports
from threading import Thread
from time import sleep

#############################################################################
# Module-level variables

# The grid of points on which we want to evaluate the field
#X, Y, Z = np.mgrid[0:10:100j, 0:10:100j, 0:10:100j]

#############################################################################
#! Threads and flow control
#!-------------------------
#!
#! There are two threads in this app:
#!  * The GUI event loop, the only thread running at the start of the program
#!  * The simulation thread, started through the GUI.
#!

# Simulation thread
class SimulationThread(Thread):
    ''' Simulation loop.
    '''
    wants_abort = False

    def run(self):
        ''' Runs the simulation loop
        '''
        # print configuration of simulator
        self.display('------ Settings ------')
        temp_str = 'Sim Area (L*W*H m): %.1f * %.1f * %.1f' %(10, 10, 01)
        self.display(temp_str)
        self.display('====== Simulation started ======')
        sim_step = 0 # Reset simulation step
        while not self.wants_abort:
            sim_step += 1
            ''' Calculate wind field '''
            ''' Calculate pollutant diffusion '''
            ''' Calculate robot position '''
            sleep(0.1)
        self.display('###### Simulation stopped ######')

#############################################################################
#! GUI elements
#!-------------
#!

# ControlPanel class, srsim UI
class ControlPanel(HasTraits):
    ''' This object is the core of the traitsUI interface. Its view is the
    right panel of the application, and it hosts the method for interaction
    between the objects and the GUI.
    '''
    ###################################
    # Panel GUI
    # ---- Control tab ----
    start_stop_simulation = Button("Start/Stop simulation")
    # Range of simulation field (length * width * height), unit: deci-meter
    field_length, field_width, field_height = Float, Float, Float
    # ---- Wind tab ----
    wind_field = Enum('uniform', 'external')
    # ---- Plume tab ----
    plume_model = Enum('farrell', 'other')
    # Simu State Text box, not in this class
    # simulator running state display
    srsim_state_display = String()

    ###################################
    # Parameter
    # Tuple of x, y, z arrays where the simulation field is sampled.
    points = Tuple(Array, Array, Array)

    ###################################
    # thread
    sim_thread = Instance(SimulationThread)

    ###################################
    # view
    view = View(VSplit(
            Group(
                Group(
                    Group(
                        Item('field_length',
                            editor = RangeEditor(   low = '1.0',
                                                    high = '50.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'L (meters)'),
                        Item('field_width',
                            editor = RangeEditor(   low = '1.0',
                                                    high = '50.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'W (meters)'),
                        Item('field_height',
                            editor = RangeEditor(   low = '1.0',
                                                    high = '50.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'H (meters)'),
                        label = 'Field Range L/W/H', show_border=True),
                    Item('start_stop_simulation', show_label = False),
                    label = 'Control', dock = 'tab'),
                Group(
                    Item(name = 'wind_field',
                        editor = EnumEditor(values = {
                            'uniform'  : '1:Uniform wind field',
                            'external' : '2:Load External wind field data',}),
                        style = 'custom'),
                    label = 'Wind', dock = 'tab'),
                Group(
                    Item(name = 'plume_model',
                        editor = EnumEditor(values = {
                            'farrell'  : '1:Farrell plume model',
                            'other'    : '2:Other plume model...',}),
                        style = 'custom'),
                    label = 'Plume', dock = 'tab'),
                layout = 'tabbed'),
            Item(name = 'srsim_state_display',
                        label = 'Simulator State', show_label=False,
                        resizable=True, springy=True, style='custom')
                )
            )

    def _start_stop_simulation_fired(self):
        ''' Callback of the "start/stop simulation" button, this starts
            the simulation thread, or kills it
        '''
        if self.sim_thread and self.sim_thread.isAlive():
            self.sim_thread.wants_abort = True
        else:
            self.sim_thread = SimulationThread()
            self.sim_thread.display = self.add_text_line
            self.sim_thread.start()

    def add_text_line(self, string):
        ''' Adds a line to Simulation State text box display
        '''
        self.srsim_state_display = (self.srsim_state_display +
                string + "\n")[0:1000]

    # Field size default: 10.0x10.0x10.0m
    def _field_length_default(self):
        field_length = 10.0
        return field_length
    def _field_width_default(self):
        field_width = 10.0
        return field_width
    def _field_height_default(self):
        field_height = 10.0
        return field_height

    def _points_default(self):
        x, y, z = np.mgrid[0:self.field_length:1, 0:self.field_width:1,
                0:self.field_height:1]
        return x, y, z

# MainWindow class, srsim UI
class MainWindow(HasTraits):
    ''' The main window
    Instructions:
        ...
        '''
    # scene of experiment area
    scene = Instance(MlabSceneModel, ())
    # control panel
    panel = Instance(ControlPanel, ())


    ##################################
    # Init
    def __init__(self, **traits):
        HasTraits.__init__(self, **traits)

    ##################################
    # Trait handlers

    ##################################
    # Private interface
    def _scene_default(self):
        scene = MlabSceneModel()
        return scene
    def _panel_default(self):
        return ControlPanel()

    ##################################
    # The UI view to show the user
    view = View(
            HSplit(
                Item(name = 'scene',
                    editor = SceneEditor(),
                    show_label = False),
                Item('panel', style = 'custom'),
                    show_labels = False,),
            resizable = True,
            title = 'Sniffer Robots Simulator 3D',
            height = 0.75, width = 0.75,
            )

##############################################################################
# Execute if running this script
if __name__ == '__main__':
    #app = ControlPanel()
    app = MainWindow()
    # open a dialog
    app.configure_traits()
