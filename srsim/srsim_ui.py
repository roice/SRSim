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
from mayavi import mlab
from mayavi.core.ui.api import MlabSceneModel, SceneEditor, MayaviScene
# Thread related imports
from threading import Thread
from time import sleep
# SRsim project
from srsim_wind_model import srsim_wind_uniform_tinv_get_uvw

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
    #scene = Instance(MlabSceneModel)



    def run(self):
        ''' Runs the simulation loop
        '''
        # print configuration of simulator
        temp_str = 'Sim Area (L*W*H): %.1f m * %.1f m * %.1f m\n\
                Grid size: 0.1 m' %(self.field_length, self.field_width,
                        self.field_height)
        self.display('====== Settings ======' + '\n' +
                temp_str + '\n' + '====== Simulation started ======')
        # Reset simulation step
        sim_step = 0
        # init simulation
        x, y, z = self.points
        ''' init wind field '''
        wind_u, wind_v, wind_w = srsim_wind_uniform_tinv_get_uvw(x, y, z, [1,2,3])
        ''' plot wind field '''
        self.wind.mlab_source.set(wind_u=wind_u)
        #flow = self.scene.mlab.vector_scatter(x, y, z, wind_u, wind_v, wind_w)
        #self.scene.mlab.test_mesh()
        #self.sim_wind_init(x,y,z,wind_u,wind_v,wind_w)
        #mlab.view(120, 60, 150)
        while not self.wants_abort:
            sim_step += 1
            ''' Update wind field '''
            ''' Update pollutant diffusion '''
            ''' Update robot position '''
            # update data for animation in simulation scene window
            sleep(1)
        self.display('###### Simulation stopped ######')

#############################################################################
#! GUI elements
#!-------------
#!

# ControlPanel class, srsim UI
# input parameter 1: scene, the mayaviscene display outside this panel
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
    # simulator running state display
    srsim_state_display = String()

    ###################################
    # Parameter
    # Tuple of x, y, z arrays where the simulation field is sampled.
    points = Tuple(Array, Array, Array)

    ###################################
    # thread and other instances
    sim_thread = Instance(SimulationThread)
    scene = Instance(MlabSceneModel)
    # The "wind" which is a Mayavi streamline module.
    wind = Instance(HasTraits)

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

    ###################################
    # Init functions

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
    def _wind_default(self):
        x, y, z = self.points
        ''' init wind field '''
        wind_u, wind_v, wind_w = srsim_wind_uniform_tinv_get_uvw(x, y, z, [1,0,0])
        ''' plot wind field '''
        f = self.scene.mlab.quiver3d(x,y,z,wind_u,wind_v,wind_w)
        return f


    ###################################
    # Widgets handlers
    def _start_stop_simulation_fired(self):
        ''' Callback of the "start/stop simulation" button, this starts
            the simulation thread, or kills it
        '''
        if self.sim_thread and self.sim_thread.isAlive():
            self.sim_thread.wants_abort = True
        else:
            # create thread
            self.sim_thread = SimulationThread()
            # link functions and parameters
            self.sim_thread.display = self.add_text_line
            self.sim_thread.field_length = self.field_length
            self.sim_thread.field_width = self.field_width
            self.sim_thread.field_height = self.field_height
            self.sim_thread.points = self.points
            self.sim_thread.wind = self.wind
            self.sim_thread.start()

    ###################################
    # Private functions
    def add_text_line(self, string):
        ''' Adds a line to Simulation State text box display
        '''
        self.srsim_state_display = (self.srsim_state_display +
                string + "\n")[0:1000]

# MainWindow class, srsim UI
class MainWindow(HasTraits):
    ''' The main window
    Instructions:
        ...
        '''
    # scene of experiment area
    scene = Instance(MlabSceneModel, ())
    # control panel
    panel = Instance(ControlPanel)

    ##################################
    # Trait handlers

    ##################################
    # Private interface
    def _panel_default(self):
        # pass self.scene to control panel
        return ControlPanel(scene = self.scene)

    ##################################
    # The UI view to show the user
    view = View(
            HSplit(
                Item(name = 'scene',
                    editor = SceneEditor(scene_class=MayaviScene),
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
