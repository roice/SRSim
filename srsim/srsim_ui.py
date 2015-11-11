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
from traitsui.api import View, Item, Group, HGroup, VGroup, HSplit, VSplit,\
        ButtonEditor, TextEditor, EnumEditor, RangeEditor, Handler,\
        CheckListEditor, BooleanEditor
from mayavi import mlab
from mayavi.core.ui.api import MlabSceneModel, SceneEditor, MayaviScene
# Thread related imports
from threading import Thread
from time import sleep
# SRsim project
import srsim_config as Config
from srsim_loop import SimulationThread
from srsim_wind import Advection
from srsim_plume import FilamentModel
from srsim_robot import Robot
# Robot drawing
from tvtk.tools import visual
# for debug
from sys import exit

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
    area_length, area_width, area_height = Int, Int, Int
    # sim time interval
    sim_dt = Float
    # sim scene display switch
    sim_scene_switch = Bool
    # simulation step count
    text_sim_step_count = Int
    # simulation time count
    text_sim_time_count = String
    # button for camera info saving
    button_save_camera_angle = Button("Save camera angle")

    # ---- Wind tab ----
    # Wind = Advective flow + Turbulent flow
    # (1) Advective flow
    # Selection of advective flow model
    enum_advection_model = Enum('constant', 'uniform', 'irrotational','ext')
    # Advection Model 1: constant advective flow field
    #  wind vector at every point is 'advection_mean'
    # Advection Model 2: uniform advective flow field
    #  wind vector at every point is the same as 'advection_mean'+stochastic variance
    # Advection Model 3: irrotational, incompressible flow
    #  wind vector at every point is different, vectors at vertex points are different
    #   'advection_mean'+stochastic variance, and vectors at six faces (cuboid flow area)
    #   are interpolated from vertex vectors (Boundary Conditions). And the rest points are
    #   calculated via laplacian equation
    advection_mean_x, advection_mean_y, \
            advection_mean_z = Float, Float, Float
    wind_colored_noise_g, wind_colored_noise_xi, wind_colored_noise_omega = \
            Float, Float, Float
    # Advection Model 3: load external advection field data

    # ---- Plume tab ----
    enum_plume_model = Enum('farrell', 'other')
    init_odor_source_pos_x, init_odor_source_pos_y,\
            init_odor_source_pos_z = Float, Float, Float
    fila_release_per_sec = Int
    fila_centerline_dispersion_sigma = Float
    fila_growth_rate = Float

    # ---- Robot tab ----
    init_robot_pos_x, init_robot_pos_y, \
            init_robot_pos_z = Float, Float, Float

    # ---- Dispay ----
    # simulator running state display
    textbox_sim_state_display = String()

    ###################################
    # ======== Data & Params ========
    # Data can be updated from outside
    # ---- simulation field grid, which is Tuple of x, y, z arrays
    grid = Tuple(Array, Array, Array)
    # ---- mean wind flow vector instantaneously
    mean_wind_vector = Tuple(Float, Float, Float)

    ###################################
    # ======== Events ========
    # results draw update event
    event_need_update_scene = Event

    ###################################
    # ======== Streamlines ========
    wind_field = Instance(HasTraits)
    odor_field = None
    odor_source = None # use visual to display a cylinder as the source
    robot_draw = None # use visual.frame to draw a robot

    ###################################
    # ======== Other ========
    # simulation thread
    sim_thread = Instance(SimulationThread)
    scene = Instance(MlabSceneModel)
    wind = Advection()
    robot = Robot()

    ###################################
    # view
    view = View(VSplit(
            Group(
                # Control tab
                Group(
                    Group(
                        Item('area_length',
                            editor = RangeEditor(   low = '1',
                                                    high = '50',
                                                    format = '%d',
                                                    mode = 'slider'),
                            label = 'L (meters)'),
                        Item('area_width',
                            editor = RangeEditor(   low = '1',
                                                    high = '50',
                                                    format = '%d',
                                                    mode = 'slider'),
                            label = 'W (meters)'),
                        Item('area_height',
                            editor = RangeEditor(   low = '1',
                                                    high = '50',
                                                    format = '%d',
                                                    mode = 'slider'),
                            label = 'H (meters)'),
                        label = 'Area Size L/W/H', show_border=True,
                        enabled_when = "params_allow_change == True"),
                    Item('sim_dt',
                        editor = RangeEditor(   low = '0.01',
                                                high = '0.1',
                                                format = '%.2f',
                                                mode = 'slider'),
                        label = 'delta t (second)',
                        enabled_when = "params_allow_change == True"),
                    HGroup(
                        Item('button_save_camera_angle', \
                                show_label = False),
                        Item('sim_scene_switch',
                        editor = BooleanEditor( mapping={
                                                    "on":True,
                                                    "off":False}),
                        label = 'Scene switch',
                        enabled_when = "params_allow_change == True")),
                    VGroup(
                        Item('text_sim_step_count',
                            editor = TextEditor(    auto_set = False,
                                                    enter_set = False),
                            label = 'Step', style = 'readonly'),
                        Item('text_sim_time_count',
                            editor = TextEditor(    auto_set = False,
                                                    enter_set = False),
                            label = 'Time', style = 'readonly'),
                        label = 'Simulation Step /& Time', show_border=True),
                    Item('button_start_stop_simulation', \
                                show_label = False),
                    label = 'Control', dock = 'tab'),
                # Wind tab
                Group(
                    Item(name = 'enum_advection_model',
                        editor = EnumEditor(values = {
                            'constant'      : '1:Constant Flow (Time Invariant)',
                            'uniform'       : '2:Uniform Flow (Time Variant)',
                            'irrotational'  : '3:Irrotational & Incompressible',
                            'ext'           : '4:Load external data',}),
                        label = 'Advection',
                        style = 'simple'),
                    Group(
                        Item('advection_mean_x',
                            editor = RangeEditor(   low = '-10.0',
                                                    high = '10.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'x (m/s):',),
                        Item('advection_mean_y',
                            editor = RangeEditor(   low = '-10.0',
                                                    high = '10.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'y (m/s):',),
                        Item('advection_mean_z',
                            editor = RangeEditor(   low = '-10.0',
                                                    high = '10.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'z (m/s):',),
                        label = 'Mean wind vector', show_border=True,
                        visible_when = "enum_advection_model == 'irrotational' or \
                                enum_advection_model == 'uniform' or enum_advection_model == 'constant'"),
                    Group(
                        Item('wind_colored_noise_g',
                            editor = RangeEditor(   low = '0.0',
                                                    high = '10.0',
                                                    format = '%.1f',
                                                    mode = 'slider'),
                            label = 'G',),
                        Item('wind_colored_noise_xi',
                            editor = RangeEditor(   low = '0.0',
                                                    high = '2.0',
                                                    format = '%.01f',
                                                    mode = 'slider'),
                            label = 'Damping',),
                        Item('wind_colored_noise_omega',
                            editor = RangeEditor(   low = '0.0',
                                                    high = '2.0',
                                                    format = '%.01f',
                                                    mode = 'slider'),
                            label = 'Bandwidth',),
                    label = 'Stochastic (colored noise) params', show_border=True,
                    visible_when = "enum_advection_model == 'irrotational' or enum_advection_model == 'uniform'"),
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
                        label = 'Odor source position (x,y,z) (m)', show_border=True),
                    Item('fila_centerline_dispersion_sigma',
                        editor = RangeEditor(   low = '0.0',
                                                high = '10.0',
                                                format = '%.1f',
                                                mode = 'slider'),
                        label = 'Sigma (m/s/sqr(Hz))',
                        visible_when = "enum_plume_model == 'farrell'"),
                    Item('fila_growth_rate',
                        editor = RangeEditor(   low = '0.0',
                                                high = '1.0',
                                                format = '%.3f',
                                                mode = 'slider'),
                        label = 'Growth rate (meter/sec)',
                        visible_when = "enum_plume_model == 'farrell'"),
                    Item('fila_release_per_sec',
                        editor = RangeEditor(   low = '1',
                                                high = '100',
                                                format = '%d',
                                                mode = 'slider'),
                        label = 'Release rate (fila number/sec)',
                        visible_when = "enum_plume_model == 'farrell'"),
                    label = 'Plume', dock = 'tab',
                    enabled_when = "params_allow_change == True"),
                # Robot tab
                Group(
                    Group(
                        Item('init_robot_pos_x',
                            editor = RangeEditor(   low = '0.0',
                                                    high_name = 'area_length',
                                                    format = '%.1f',
                                                    mode = 'auto'),
                            label = 'x (m):',),
                        Item('init_robot_pos_y',
                            editor = RangeEditor(   low = '0.0',
                                                    high_name = 'area_width',
                                                    format = '%.1f',
                                                    mode = 'auto'),
                            label = 'y (m):',),
                        Item('init_robot_pos_z',
                            editor = RangeEditor(   low = '0.0',
                                                    high_name = 'area_height',
                                                    format = '%.1f',
                                                    mode = 'auto'),
                            label = 'z (m):',),
                        label = 'Robot position (x,y,z) (m)', show_border=True),
                    label = 'Robot', dock = 'tab',
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

    def _sim_dt_default(self):
        return Config.get_dt()

    def _sim_scene_switch_default(self):
        return True

    def _params_allow_change_default(self):
        return True

    def _enum_advection_model_default(self):
        model = Config.get_wind_model()
        return model

    def _advection_mean_x_default(self):
        return self.mean_wind_vector[0]
    def _advection_mean_y_default(self):
        return self.mean_wind_vector[1]
    def _advection_mean_z_default(self):
        return self.mean_wind_vector[2]

    def _wind_colored_noise_g_default(self):
        noise_params = Config.get_wind_colored_noise_params()
        return noise_params[0]
    def _wind_colored_noise_xi_default(self):
        noise_params = Config.get_wind_colored_noise_params()
        return noise_params[1]
    def _wind_colored_noise_omega_default(self):
        noise_params = Config.get_wind_colored_noise_params()
        return noise_params[2]


    def _init_odor_source_pos_x_default(self):
        pos = Config.get_odor_source_pos()
        return pos[0]
    def _init_odor_source_pos_y_default(self):
        pos = Config.get_odor_source_pos()
        return pos[1]
    def _init_odor_source_pos_z_default(self):
        pos = Config.get_odor_source_pos()
        return pos[2]

    def _init_robot_pos_x_default(self):
        pos = Config.get_robot_init_pos()
        return pos[0]
    def _init_robot_pos_y_default(self):
        pos = Config.get_robot_init_pos()
        return pos[1]
    def _init_robot_pos_z_default(self):
        pos = Config.get_robot_init_pos()
        return pos[2]


    def _grid_default(self):
        gsize = Config.get_wind_grid_size()
        x, y, z = np.mgrid[gsize/2.0:self.area_length:gsize, gsize/2.0:self.area_width:gsize,
                gsize/2.0:self.area_height:gsize]
        return x, y, z

    def _mean_wind_vector_default(self):
        vector = Config.get_mean_wind_vector()
        return vector[0], vector[1], vector[2]

    def _fila_centerline_dispersion_sigma_default(self):
        params = Config.get_plume_model_params()
        return params[0]

    def _fila_growth_rate_default(self):
        params = Config.get_plume_model_params()
        return params[1]

    def _fila_release_per_sec_default(self):
        params = Config.get_plume_model_params()
        return params[2]

    def _text_sim_step_count_default(self):
        return 0

    def _text_sim_time_count_default(self):
        return '0.0 s'

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
        cn_p = Config.get_wind_colored_noise_params()
        self.wind.G = cn_p[0]
        self.wind.wind_damping = cn_p[1]
        self.wind.wind_bandwidth = cn_p[2]
        # init scene
        #   init wind vector field obj
        self.wind.uniform_tinv()
        u, v, w = self.wind.wind_vector_field
        x, y, z = self.grid
        self.wind_field = self.scene.mlab.quiver3d(x, y, z, u, v, w, reset_zoom=False)
        #   init odor source pos obj
        visual.set_viewer(self.scene) # tell visual to use this scene as the viewer
        op = Config.get_odor_source_pos() # get init odor source pos
        #self.odor_source = visual.cylinder(pos=(op[0], op[1], 0), axis = (0, 0, 1), \
        #        length = op[2], radius = 0.1)
        self.odor_source = visual.box(pos=(op[0], op[1], op[2]/2.0), length = 0.1, \
                height = 0.1, width = op[2], color = (0x05/255.0, 0x5f/255.0,0x58/255.0), )
        #   init robot drawing obj
        rp = Config.get_robot_init_pos()
        self.func_init_robot_shape(rp)
        # axes and outlines
        self.func_init_axes_outline()
        # camera view
        self.scene.mlab.view(*Config.get_camera_view())

    @on_trait_change('area_length, area_width, area_height')
    def change_area_size(self):
        # change mesh grid to new shape
        gsize = Config.get_wind_grid_size() # get advection grid size
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
        self.wind_field.remove()
        self.wind_field = self.scene.mlab.quiver3d(x, y, z, u, v ,w, reset_zoom=False)
        # draw axes & outline
        self.func_init_axes_outline()
        self.scene.disable_render = False
        # save area setting to global settings
        Config.set_sim_area_size([self.area_length, self.area_width, self.area_height])

    @on_trait_change('sim_dt')
    def change_dt(self):
        # save sim dt to global settings
        Config.set_dt(self.sim_dt)

    @on_trait_change('enum_advection_model')
    def change_advection_model_selection(self):
        # save selection to global settings
        Config.set_wind_model(self.enum_advection_model)

    @on_trait_change('advection_mean_x, advection_mean_y, \
            advection_mean_z')
    def change_advection_mean_vector(self):
        self.mean_wind_vector = self.advection_mean_x,\
                self.advection_mean_y, self.advection_mean_z
        #save mean wind vector setting to global settings
        Config.set_mean_wind_vector(list(self.mean_wind_vector))
        # update self.wind.mean_flow
        self.wind.mean_flow = list(self.mean_wind_vector)
        # draw wind vector field
        self.wind.uniform_tinv()
        u, v, w = self.wind.wind_vector_field
        self.wind_field.mlab_source.set(u=u, v=v, w=w)

    @on_trait_change('wind_colored_noise_g, wind_colored_noise_xi, wind_colored_noise_omega')
    def change_wind_colored_noise_params(self):
        self.wind.G = self.wind_colored_noise_g
        self.wind.wind_damping = self.wind_colored_noise_xi
        self.wind_wind_bandwidth = self.wind_colored_noise_omega
        # save wind colored noise params to global settings
        Config.set_wind_colored_noise_params([self.wind_colored_noise_g, \
                self.wind_colored_noise_xi, self.wind_colored_noise_omega])

    @on_trait_change('init_odor_source_pos_x, init_odor_source_pos_y, init_odor_source_pos_z')
    def change_init_odor_source_pos(self):
        # change the pos of odor source obj
        x, y, z = self.init_odor_source_pos_x,\
                self.init_odor_source_pos_y, self.init_odor_source_pos_z
        self.odor_source.pos = (x, y, z/2.0)
        self.odor_source.width = z
        # save pos setting to global settings
        Config.set_odor_source_pos([x, y, z])

    @on_trait_change('init_robot_pos_x, init_robot_pos_y, init_robot_pos_z')
    def change_init_robot_pos(self):
        # change the pos of robot drawing obj
        x, y, z = self.init_robot_pos_x, self.init_robot_pos_y, self.init_robot_pos_z
        self.robot_draw.pos = (x, y, z)
        # save pos setting to global settings
        Config.set_robot_init_pos([x, y, z])

    @on_trait_change('fila_release_per_sec, fila_centerline_dispersion_sigma, fila_growth_rate')
    def change_farrell_params(self):
        # save farrell params to global settings
        Config.set_plume_model_params([self.fila_centerline_dispersion_sigma, \
                self.fila_growth_rate, self.fila_release_per_sec])

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
            # check if need re-init scene
            if self.sim_thread: # if sim_thread != None, i.e., if sim_thread once excecuted
                # re-init scene
                self.odor_field.remove()
                self.wind_field.remove()
                self.wind.uniform_tinv()
                u, v, w = self.wind.wind_vector_field
                x, y, z = self.grid
                self.wind_field = self.scene.mlab.quiver3d(x, y, z, u, v, w, reset_zoom=False)
            # print simulator settings
            self.textbox_sim_state_display = '' # clear display
            sim_area_str = 'Sim Area (L*W*H): %.1f m * %.1f m * %.1f m\n\
                    Grid size: %.1f m' %(self.area_length, self.area_width,
                            self.area_height, Config.get_wind_grid_size())
            sim_wind_str = 'Wind type: ' + str(self.enum_advection_model) + '\n' \
                    + '  Mean wind vector = ' + str(self.mean_wind_vector)
            if self.enum_advection_model == 'irrotational':
                sim_wind_str += '\n' + '  Colored noise Params: ' + '\n' \
                        + '    G = ' + str(self.wind.G) + '\n' \
                        + '    wind_damping = ' + str(self.wind.wind_damping) + '\n' \
                        + '    wind_bandwidth = ' + str(self.wind.wind_bandwidth)
            self.add_text_line('====== Settings ======' + '\n' +
                    sim_area_str + '\n' + sim_wind_str + '\n' + '====== Simulation started ======')
            # create & start simulation thread
            self.sim_thread = SimulationThread()
            # sim visual update function
            self.sim_thread.update_scene = self.func_event_update_scene
            # sim step count function
            self.sim_thread.count_sim_step = self.func_sim_step_count
            # clear count
            self.text_sim_step_count = 0
            self.text_sim_time_count = '0.0 s'
            # sim textbox state display function
            self.sim_thread.display = self.add_text_line
            # wind sim instance
            self.sim_thread.wind = self.wind
            self.wind.dt = self.sim_dt
            # send wind model selection to sim_thread
            self.sim_thread.wind_model = self.enum_advection_model
            # plume sim instance
            if self.enum_plume_model == 'farrell':
                self.sim_thread.plume = FilamentModel()
                self.sim_thread.plume.odor_source_pos = [self.init_odor_source_pos_x,\
                    self.init_odor_source_pos_y, self.init_odor_source_pos_z]
                self.sim_thread.plume.sim_area_size = [self.area_length, self.area_width, self.area_height]
                self.sim_thread.plume.dt = self.sim_dt
                self.sim_thread.plume.adv_mesh = self.grid
                self.sim_thread.plume.adv_xyz_n = self.wind.xyz_n
                self.sim_thread.plume.adv_gsize = self.wind.gsize
                self.sim_thread.plume.vm_sigma = self.fila_centerline_dispersion_sigma
                self.sim_thread.plume.fila_growth_rate = self.fila_growth_rate
                self.sim_thread.plume.fila_number_per_sec = self.fila_release_per_sec
                self.odor_field = self.scene.mlab.points3d([0.0], [0.0], [0.0], [0.0], \
                        color = (0,0,0), scale_factor=1, reset_zoom=False)
            else:
                exit('Error: Other plume model not supported yet...')
            # robot instance
            self.sim_thread.robot = self.robot
            #  robot init position
            self.robot.robot_pos = [self.init_robot_pos_x, self.init_robot_pos_y, self.init_robot_pos_z]
            #  odor sampling routine
            self.robot.odor_sampling = self.sim_thread.plume.odor_conc_value_sampling
            # start simulation thread
            self.sim_thread.start()

    def _event_need_update_scene_fired(self):
        # if scene switch is turned on (default)
        if self.sim_scene_switch == True:
            self.scene.disable_render = True
            self.func_update_scene()
            self.scene.disable_render = False

    def _button_save_camera_angle_fired(self):
        cam = self.scene.mlab.view()
        Config.set_camera_view(cam)

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
        # count sim step
        self.text_sim_step_count += 1
        # count sim time
        self.text_sim_time_count = '%.1f s' %(self.text_sim_step_count*self.sim_dt)

    # draw axes & outlines on current scene
    def func_init_axes_outline(self):
        self.scene.mlab.axes( xlabel = 'X East (m)', ylabel = 'Y North (m)',
                zlabel = 'Z Up (m)', ranges = [0, self.area_length, 0,
                    self.area_width, 0, self.area_height],)
        self.scene.mlab.outline(extent=[0, self.area_length, 0,
            self.area_width, 0, self.area_height],)

    # init robot shape drawing
    def func_init_robot_shape(self, robot_pos):
        # draw 4 propellers
        d = 0.145 # diameter of propeller is 14.5 cm
        w = 0.16 # wheelbase is 16 cm (shortest distance between centers of two adjacent propellers)
        p1 = visual.ring(pos=(w/2.0, w/2.0, 0), axis = (0, 0, 1), \
                radius = d/2.0, thickness = 0.01, color = (1.0, 0, 0)) # Red
        p2 = visual.ring(pos=(w/2.0, -w/2.0, 0), axis = (0, 0, 1), \
                radius = d/2.0, thickness = 0.01, color = (0, 0, 1.0)) # Blue
        p3 = visual.ring(pos=(-w/2.0, -w/2.0, 0), axis = (0, 0, 1), \
                radius = d/2.0, thickness = 0.01, color = (0, 0, 1.0)) # Blue
        p4 = visual.ring(pos=(-w/2.0, w/2.0, 0), axis = (0, 0, 1), \
                radius = d/2.0, thickness = 0.01, color = (1.0, 0, 0)) # Red
        # robot shape is a frame composites different objects
        self.robot_draw = visual.frame(p1, p2, p3, p4)
        self.robot_draw.pos = (robot_pos[0], robot_pos[1], robot_pos[2])

    # update scene
    def func_update_scene(self):
        # update wind field display
        u, v, w = self.sim_thread.wind_vector_field
        self.wind_field.mlab_source.set(u=u, v=v, w=w)
        # update odor field display
        fila = self.sim_thread.plume_snapshot
        # update obj
        self.odor_field.mlab_source.reset(x=fila['x'], y=fila['y'], z=fila['z'], scalars=fila['r']*50)
        self.odor_field.mlab_source.set(x=fila['x']) # to trig display refresh

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
