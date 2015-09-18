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
        Range, Array, Tuple, Int, Float, Event, on_trait_change
from traitsui.api import View, Item, Group, HSplit, VSplit,\
        ButtonEditor, TextEditor, EnumEditor, RangeEditor, Handler
from mayavi import mlab
from mayavi.core.ui.api import MlabSceneModel, SceneEditor, MayaviScene
# Thread related imports
from threading import Thread
from time import sleep
# SRsim project
from srsim_loop import SimulationThread
from srsim_wind_model import srsim_wind_uniform_tinv_get_uvw

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
    # ======== Panel GUI ========
    # ---- Control tab ----
    button_start_stop_simulation = Button("Start/Stop simulation")
    # Range of simulation area (length * width * height), unit: meter
    area_length, area_width, area_height = Float, Float, Float
    # simulation step count
    text_sim_step_count = Int
    #sim_thread_test = Instance(SimulationThread)
    #text_sim_step_count = DelegatesTo("sim_thread_test", prefix="step_count")
    # ---- Wind tab ----
    enum_wind_field_model = Enum('uniform', 'external')
    # ---- Plume tab ----
    enum_plume_model = Enum('farrell', 'other')
    # simulator running state display
    textbox_sim_state_display = String()

    ###################################
    # ======== Data & Params ========
    # Data can be updated from outside
    # ---- simulation field grid, which is Tuple of x, y, z arrays
    grid = Tuple(Array, Array, Array)
    # ---- wind vector field
    data_wind_field = Tuple(Array, Array, Array)
    # ---- plume scalar field
    #data_odor_field = Tuple(Array)

    ###################################
    # ======== Events ========
    # results draw update event
    event_need_update_scene = Event

    ###################################
    # ======== Streamlines ========
    sl_wind = Instance(HasTraits)

    ###################################
    # ======== Other ========
    # simulation thread
    sim_thread = Instance(SimulationThread)


    ###################################
    # view
    view = View(VSplit(
            Group(
                Group(
                    Group(
                        Item('area_length',
                            editor = RangeEditor(   low = '1.0',
                                                    high = '50.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'L (meters)'),
                        Item('area_width',
                            editor = RangeEditor(   low = '1.0',
                                                    high = '50.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'W (meters)'),
                        Item('area_height',
                            editor = RangeEditor(   low = '1.0',
                                                    high = '50.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'H (meters)'),
                        label = 'Area Size L/W/H', show_border=True),
                    Group(
                        Item('text_sim_step_count',
                            editor = TextEditor(    auto_set = False,
                                                    enter_set = False),
                            label = 'Step', style = 'readonly'),
                        label = 'Simulation Step count', show_border=True),
                    Item('button_start_stop_simulation', show_label = False),
                    label = 'Control', dock = 'tab'),
                Group(
                    Item(name = 'enum_wind_field_model',
                        editor = EnumEditor(values = {
                            'uniform'  : '1:Uniform wind field',
                            'external' : '2:Load External wind field data',}),
                        style = 'custom'),
                    label = 'Wind', dock = 'tab'),
                Group(
                    Item(name = 'enum_plume_model',
                        editor = EnumEditor(values = {
                            'farrell'  : '1:Farrell plume model',
                            'other'    : '2:Other plume model...',}),
                        style = 'custom'),
                    label = 'Plume', dock = 'tab'),
                layout = 'tabbed'),
            Item(name = 'textbox_sim_state_display',
                        label = 'Simulator State', show_label=False,
                        resizable=True, springy=True, style='custom')
                )
            )

    ###################################
    # ======== Init functions ========
    def __init__(self, scene):
        HasTraits.__init__(self)
        # pass 'scene' from MainWindow class
        self.scene = scene

    # Area size default: 10.0x10.0x10.0m
    def _area_length_default(self):
        area_length = 10.0
        return area_length
    def _area_width_default(self):
        area_width = 10.0
        return area_width
    def _area_height_default(self):
        area_height = 10.0
        return area_height

    def _text_sim_step_count(self):
        c = 0
        return c

    def _grid_default(self):
        x, y, z = np.mgrid[0:self.area_length:1, 0:self.area_width:1,
                0:self.area_height:1]
        return x, y, z

    def _data_wind_field_default(self):
        x, y, z = self.grid
        u = np.ones_like(x)
        v = np.ones_like(y)
        w = np.ones_like(z)
        return u, v, w


    ###################################
    # ======== Listening ========
    def _button_start_stop_simulation_fired(self):
        ''' Callback of the "start/stop simulation" button, this starts
            the simulation thread, or kills it
        '''
        if self.sim_thread and self.sim_thread.isAlive():
            # kill simulation thread if it's running
            self.sim_thread.wants_abort = True
            temp_str = 'step = %d' %self.text_sim_step_count
            self.add_text_line(temp_str)
        else:
            # print simulator settings
            temp_str = 'Sim Area (L*W*H): %.1f m * %.1f m * %.1f m\n\
                    Grid size: 0.1 m' %(self.area_length, self.area_width,
                            self.area_height)
            self.add_text_line('====== Settings ======' + '\n' +
                    temp_str + '\n' + '====== Simulation started ======')
            # init scene
            x, y, z = self.grid
            wind_u, wind_v, wind_w = self.data_wind_field
            wind_u, wind_v, wind_w = srsim_wind_uniform_tinv_get_uvw(x, y, z, [1,0,0])
            self.sl_wind = self.scene.mlab.quiver3d(x,y,z,wind_u,wind_v,wind_w)
            # create & start simulation thread
            self.sim_thread = SimulationThread()
            self.sim_thread.update_scene = self.func_event_update_scene
            self.sim_thread.count_sim_step = self.func_sim_step_count
            self.sim_thread.display = self.add_text_line
            self.sim_thread.grid = self.grid
            self.sim_thread.data_wind_field = self.data_wind_field
            self.sim_thread.start()


    def _event_need_update_scene_fired(self):
        wind_u, wind_v, wind_w = self.data_wind_field
        self.sl_wind.mlab_source.set(u=wind_u, v=wind_v, w=wind_w)

    ###################################
    # Private functions
    def add_text_line(self, string):
        ''' Adds a line to Simulation State text box display
        '''
        self.textbox_sim_state_display = (self.textbox_sim_state_display +
                string + "\n")[0:1000]
    # trigger scene update event
    def func_event_update_scene(self):
        self.event_need_update_scene = True

    def func_sim_step_count(self):
        self.text_sim_step_count += 1

class MainWindowHandler(Handler):
    def close(self, info, is_OK):
        # close panel.sim_thread thread
        if(info.object.panel.sim_thread
                and info.object.panel.sim_thread.isAlive()):
            info.object.panel.sim_thread.wants_abort = True
            while info.object.panel.sim_thread.isAlive():
                sleep(0.1)
        return True

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
            handler = MainWindowHandler(),
            )

##############################################################################
# Execute if running this script
if __name__ == '__main__':
    #app = ControlPanel()
    app = MainWindow()
    # open a dialog
    app.configure_traits()
