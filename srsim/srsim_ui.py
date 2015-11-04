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
        Range, Array, Tuple, Int, Float, Event, on_trait_change, Bool, List,\
        Str
from traitsui.api import View, Item, Group, HSplit, VSplit,\
        ButtonEditor, TextEditor, EnumEditor, RangeEditor, Handler,\
        CheckListEditor
from mayavi import mlab
from mayavi.core.ui.api import MlabSceneModel, SceneEditor, MayaviScene
# Thread related imports
from threading import Thread
from time import sleep
# SRsim project
import srsim_config as Config
from srsim_loop import SimulationThread
from srsim_wind import Advection

#############################################################################
#! Global parameters
#!------------------
#!
# see config file settings.cfg and srsim module srsim_config.py

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
    # bool switch, to enable/disable params changing
    params_allow_change = Bool
    # ---- Control tab ----
    button_start_stop_simulation = Button("Start/Stop simulation")
    # Range of simulation area (length * width * height), unit: meter
    area_length, area_width, area_height = Float, Float, Float
    # simulation step count
    text_sim_step_count = Int

    # ---- Wind tab ----
    # Wind = Advective flow + Turbulent flow
    # (1) Advective flow
    # Selection of advective flow model
    enum_advection_model = Enum('uniform','irrotational','ext')
    # Advection Model 1: uniform advective flow field
    uniform_advection_vector_x, uniform_advection_vector_y, \
            uniform_advection_vector_z = Float, Float, Float
    # Advection Model 2: irrotational, incompressible flow
    irrotational_advection_mean_x, irrotational_advection_mean_y, \
            irrotational_advection_mean_z = Float, Float, Float
    # Advection Model 3: load external advection field data

    # ---- Plume tab ----
    enum_plume_model = Enum('farrell', 'other')
    init_odor_source_pos_x, init_odor_source_pos_y,\
            init_odor_source_pos_z = Float, Float, Float
    # simulator running state display
    textbox_sim_state_display = String()

    ###################################
    # ======== Data & Params ========
    # Data can be updated from outside
    # ---- simulation field grid, which is Tuple of x, y, z arrays
    grid = Tuple(Array, Array, Array)
    # ---- mean wind flow vector instantaneously
    mean_wind_vector = Tuple(Float, Float, Float)
    # ---- plume scalar field
    data_odor_field = Array
    # ---- masked odor field grid, for quick display
    grid_odor_masked = Array
    # ---- odor source position tuple
    odor_source_pos = Tuple(Float, Float, Float)

    ###################################
    # ======== Events ========
    # results draw update event
    event_need_update_scene = Event

    ###################################
    # ======== Streamlines ========
    wind_field = Instance(HasTraits)
    odor_field = Instance(HasTraits)

    ###################################
    # ======== Other ========
    # simulation thread
    sim_thread = Instance(SimulationThread)
    scene = Instance(MlabSceneModel)
    wind = Advection()


    ###################################
    # view
    view = View(VSplit(
            Group(
                # Control tab
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
                        label = 'Area Size L/W/H', show_border=True,
                        enabled_when = "params_allow_change == True"),
                    Group(
                        Item('text_sim_step_count',
                            editor = TextEditor(    auto_set = False,
                                                    enter_set = False),
                            label = 'Step', style = 'readonly'),
                        label = 'Simulation Step count', show_border=True),
                    Item('button_start_stop_simulation', show_label = False),
                    label = 'Control', dock = 'tab'),
                # Wind tab
                Group(
                    Item(name = 'enum_advection_model',
                        editor = EnumEditor(values = {
                            'uniform'       : '1:uniform',
                            'irrotational'  : '2:irrotational & incompressible',
                            'ext'           : '3:load external data',}),
                        label = 'Advection',
                        style = 'simple'),
                    Group(
                        Item('uniform_advection_vector_x',
                            editor = RangeEditor(   low = '-10.0',
                                                    high = '10.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'x (m/s):',),
                        Item('uniform_advection_vector_y',
                            editor = RangeEditor(   low = '-10.0',
                                                    high = '10.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'y (m/s):',),
                        Item('uniform_advection_vector_z',
                            editor = RangeEditor(   low = '-10.0',
                                                    high = '10.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'z (m/s):',),
                        label = 'Uniform advection vector', show_border=True,
                        visible_when = "enum_advection_model == 'uniform'"),
                    Group(
                        Item('irrotational_advection_mean_x',
                            editor = RangeEditor(   low = '-10.0',
                                                    high = '10.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'x (m/s):',),
                        Item('irrotational_advection_mean_y',
                            editor = RangeEditor(   low = '-10.0',
                                                    high = '10.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'y (m/s):',),
                        Item('irrotational_advection_mean_z',
                            editor = RangeEditor(   low = '-10.0',
                                                    high = '10.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'z (m/s):',),
                        label = 'Mean wind vector', show_border=True,
                        visible_when = "enum_advection_model == 'irrotational'"),
                    label = 'Wind', dock = 'tab',
                    enabled_when = "params_allow_change == True"),
                # Plume tab
                Group(
                    Item(name = 'enum_plume_model',
                        editor = EnumEditor(values = {
                            'farrell'  : '1:Farrell plume model',
                            'other'    : '2:Other plume model...',}),
                        style = 'custom'),
                    Group(
                        Item('init_odor_source_pos_x',
                            editor = RangeEditor(   low = '0.0',
                                                    high_name = 'area_length',
                                                    format = '%.1f',
                                                    mode = 'auto'),
                            label = 'x (m):',),
                        Item('init_odor_source_pos_y',
                            editor = RangeEditor(   low = '0.0',
                                                    high_name = 'area_width',
                                                    format = '%.1f',
                                                    mode = 'auto'),
                            label = 'y (m):',),
                        Item('init_odor_source_pos_z',
                            editor = RangeEditor(   low = '0.0',
                                                    high_name = 'area_height',
                                                    format = '%.1f',
                                                    mode = 'auto'),
                            label = 'z (m):',),
                        label = 'Init odor source position (x,y,z) (m)', show_border=True),
                    label = 'Plume', dock = 'tab',
                    enabled_when = "params_allow_change == True"),
                layout = 'tabbed'),
            Item(name = 'textbox_sim_state_display',
                        label = 'Simulator State', show_label=False,
                        resizable=True, springy=True, style='custom')
                )
            )

    ###################################
    # ======== Init functions ========
    def __init__(self, **traits):
        HasTraits.__init__(self, **traits)

    # Area size default: 10.0x10.0x10.0m
    def _area_length_default(self):
        size = Config.get_sim_area_size()
        return size[0]
    def _area_width_default(self):
        size = Config.get_sim_area_size()
        return size[1]
    def _area_height_default(self):
        size = Config.get_sim_area_size()
        return size[2]

    def _params_allow_change_default(self):
        return True

    def _enum_advection_model_default(self):
        model = Config.get_wind_model()
        return model

    def _uniform_advection_vector_x_default(self):
        return self.mean_wind_vector[0]
    def _uniform_advection_vector_y_default(self):
        return self.mean_wind_vector[1]
    def _uniform_advection_vector_z_default(self):
        return self.mean_wind_vector[2]

    def _irrotational_advection_mean_x_default(self):
        return self.mean_wind_vector[0]
    def _irrotational_advection_mean_y_default(self):
        return self.mean_wind_vector[1]
    def _irrotational_advection_mean_z_default(self):
        return self.mean_wind_vector[2]

    def _init_odor_source_pos_x_default(self):
        pos = Config.get_odor_source_pos()
        return pos[0]
    def _init_odor_source_pos_y_default(self):
        pos = Config.get_odor_source_pos()
        return pos[1]
    def _init_odor_source_pos_z_default(self):
        pos = Config.get_odor_source_pos()
        return pos[2]


    def _grid_default(self):
        gsize = Config.get_wind_grid_size()
        x, y, z = np.mgrid[gsize/2.0:self.area_length:gsize, gsize/2.0:self.area_width:gsize,
                gsize/2.0:self.area_height:gsize]
        return x, y, z

    def _mean_wind_vector_default(self):
        vector = Config.get_mean_wind_vector()
        return vector[0], vector[1], vector[2]

    def _data_odor_field_default(self):
        x, y, z = self.grid
        o = np.ones_like(x)
        return o

    def _text_sim_step_count_default(self):
        c = 0
        return c

    def _odor_field_default(self):
        x = self.init_odor_source_pos_x
        y = self.init_odor_source_pos_y
        z = self.init_odor_source_pos_z
        o = [1.0]
        odor_field = self.scene.mlab.points3d(x, y, z, o, colormap = "cool",
                opacity = 0.3, transparent = True, scale_factor=1.0)
        return odor_field

    ###################################
    # ======== Listening ========
    @on_trait_change('scene.activated')
    def init_scene(self):
        # init params of wind model
        l, w, h = np.array(self.grid).shape[1:4] # 1 2 3
        self.wind.xyz_n = [l, w, h]
        gsize = Config.get_wind_grid_size()
        self.wind.gsize = gsize
        self.wind.mean_flow = list(self.mean_wind_vector)
        # init scene
        self.wind.uniform_tinv()
        u, v, w = self.wind.wind_vector_field
        x, y, z = self.grid
        self.wind_field = self.scene.mlab.quiver3d(x, y, z, u, v, w)
        # axes and outlines
        self.func_init_axes_outline()
        # compute odor field(init)
        #self.func_odor_field_init()
        o_m = [1.0]
        self.odor_field.mlab_source.set(s=o_m)

    @on_trait_change('area_length, area_width, area_height')
    def change_area_size(self):
        # change mesh grid to new shape
        gsize = Config.get_wind_grid_size()
        x, y, z = np.mgrid[gsize/2.0:self.area_length:gsize, gsize/2.0:self.area_width:gsize,
                gsize/2.0:self.area_height:gsize]
        self.grid = x, y, z
        # update self.wind.xyz_n
        self.wind.xyz_n = np.array(self.grid).shape[1:4] # 1 2 3

        # re-init plotting
        # speed up plotting
        self.scene.disable_render = True
        # clean figure
        self.scene.mlab.clf()
        # draw wind vector field
        self.wind.uniform_tinv()
        u, v, w = self.wind.wind_vector_field
        self.wind_field = self.scene.mlab.quiver3d(x, y, z, u, v ,w)
        # draw axes & outline
        self.func_init_axes_outline()
        self.scene.disable_render = False
        # save area setting to global settings
        Config.set_sim_area_size([self.area_length, self.area_width, self.area_height])

    @on_trait_change('enum_advection_model, uniform_advection_vector_x, \
            uniform_advection_vector_y, uniform_advection_vector_z, \
            irrotational_advection_mean_x, irrotational_advection_mean_y, \
            irrotational_advection_mean_z')
    def change_advection_model_selection(self):
        if self.enum_advection_model == 'uniform':
            self.mean_wind_vector = self.uniform_advection_vector_x,\
                self.uniform_advection_vector_y, self.uniform_advection_vector_z
            #save mean wind vector setting to global settings
            Config.set_mean_wind_vector(list(self.mean_wind_vector))
        elif self.enum_advection_model == 'irrotational':
            self.mean_wind_vector = self.irrotational_advection_mean_x,\
                self.irrotational_advection_mean_y, self.irrotational_advection_mean_z
            #save mean wind vector setting to global settings
            Config.set_mean_wind_vector(list(self.mean_wind_vector))
        # update self.wind.mean_flow
        self.wind.mean_flow = list(self.mean_wind_vector)
        # draw wind vector field
        self.wind.uniform_tinv()
        u, v, w = self.wind.wind_vector_field
        self.wind_field.mlab_source.set(u=u, v=v, w=w)
        # save selection to global settings
        Config.set_wind_model(self.enum_advection_model)

    @on_trait_change('init_odor_source_pos_x, init_odor_source_pos_y, init_odor_source_pos_z')
    def change_init_odor_source_pos(self):
        # change odor source position
        x, y, z = self.init_odor_source_pos_x,\
                self.init_odor_source_pos_y, self.init_odor_source_pos_z
        o = [1.0]
        self.odor_field.mlab_source.set(x=x, y=y, z=z)
        # remove previous drawing of odor source
        #self.odor_field.remove()
        # re-draw odor source
        #self.odor_field = self.scene.mlab.points3d(x, y, z, o, colormap = "cool",
        #        opacity = 0.3, transparent = True, scale_factor=1.0)
        # save pos setting to global settings
        Config.set_odor_source_pos([self.init_odor_source_pos_x,\
                self.init_odor_source_pos_y, self.init_odor_source_pos_z])


    def _button_start_stop_simulation_fired(self):
        ''' Callback of the "start/stop simulation" button, this starts
            the simulation thread, or kills it
        '''

        if self.sim_thread and self.sim_thread.isAlive():
            # kill simulation thread if it's running
            self.sim_thread.wants_abort = True
            # enable area size changing item (GUI)
            self.params_allow_change = True
        else:
            # disable area size changing item (GUI)
            self.params_allow_change = False
            # print simulator settings
            temp_str = 'Sim Area (L*W*H): %.1f m * %.1f m * %.1f m\n\
                    Grid size: %.1f m' %(self.area_length, self.area_width,
                            self.area_height, Config.get_wind_grid_size())
            self.add_text_line('====== Settings ======' + '\n' +
                    temp_str + '\n' + '====== Simulation started ======')
            # create & start simulation thread
            self.sim_thread = SimulationThread()
            # sim visual update function
            self.sim_thread.update_scene = self.func_event_update_scene
            # sim step count function
            self.sim_thread.count_sim_step = self.func_sim_step_count
            # clear count
            self.text_sim_step_count = 0
            # sim textbox state display function
            self.sim_thread.display = self.add_text_line
            # wind sim instance
            self.sim_thread.wind = self.wind
            # copy self.grid
            self.sim_thread.grid = self.grid
            # send wind model selection to sim_thread
            self.sim_thread.wind_model = self.enum_advection_model
            # send odor source pos to sim_thread
            self.odor_source_pos = self.init_odor_source_pos_x,\
                self.init_odor_source_pos_y, self.init_odor_source_pos_z
            self.sim_thread.odor_source_pos = self.odor_source_pos
            self.sim_thread.start()

    def _event_need_update_scene_fired(self):
        self.scene.disable_render = True
        # update wind field display
        u, v, w = self.sim_thread.wind_vector_field
        #print 'u.shape = ' + str(u.shape)
        #print 'u = ' + str(u)
        self.wind_field.mlab_source.set(u=u, v=v, w=w)
        self.scene.disable_render = False



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

    # draw axes & outlines on current scene
    def func_init_axes_outline(self):
        self.scene.mlab.axes( xlabel = 'X East (m)', ylabel = 'Y North (m)',
                zlabel = 'Z Up (m)', ranges = [0, self.area_length, 0,
                    self.area_width, 0, self.area_height],)
        self.scene.mlab.outline(extent=[0, self.area_length, 0,
            self.area_width, 0, self.area_height],)

    # slice 3d mesh matrix to smaller matrix for quick display
    '''
    input:
        interval ---- slice interval, must be even number
                        interval/2:len(xyz_grids):interval
    '''
    def func_slice_mesh_matrix_3d(self, mesh_grid, interval):
        x, y, z = mesh_grid
        x_s = x[interval/2:x.shape[0]:interval,
                interval/2:x.shape[1]:interval, interval/2:x.shape[2]:interval]
        y_s = y[interval/2:y.shape[0]:interval,
                interval/2:y.shape[1]:interval, interval/2:y.shape[2]:interval]
        z_s = z[interval/2:z.shape[0]:interval,
                interval/2:z.shape[1]:interval, interval/2:z.shape[2]:interval]
        return x_s, y_s, z_s

    # slice 3d mesh matrix to x/y/z array, reverse process of mgrid
    def func_slice_mesh_matrix2array(self, mesh_grid, interval):
        x, y, z = mesh_grid
        x_s = x[interval/2:x.shape[0]:interval, 0, 0]
        y_s = y[0, interval/2:y.shape[1]:interval, 0]
        z_s = z[0, 0, interval/2:z.shape[2]:interval]
        return x_s, y_s, z_s

    # init odor (concentration) field
    def func_odor_field_init(self):
        self.data_odor_field = srsim_plume(self.enum_plume_model, self.grid,
                self.data_wind_field)

class MainWindowHandler(Handler):
    def close(self, info, is_OK):
        # close panel.sim_thread thread
        if(info.object.panel.sim_thread
                and info.object.panel.sim_thread.isAlive()):
            info.object.panel.sim_thread.wants_abort = True
            while info.object.panel.sim_thread.isAlive():
                sleep(0.1)
        # save settings to file settings.cfg`
        Config.save_settings()
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
        # load settings from file settings.cfg
        Config.load_settings()
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
