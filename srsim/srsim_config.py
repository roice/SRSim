#!/usr/bin/python
# coding=utf-8
#
# sniffer robots simulator
#                        Save & get settings
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

'''srsim config

Documentation and tests are included in ...
'''

import ConfigParser
import sys, os
import numpy as np

class Settings:
    # [setup]
    # simulation area size default values [l*w*h] meters
    sim_area_size = 'None'
    # odor source position
    odor_source_pos = 'None'
    # sim time interval
    dt = 'None'

    # [wind]
    # wind model
    wind_model = 'None'
    # grid size of advective flow field simulation, must be float (meter)
    wind_grid_size = 'None'
    # mean advection vector
    mean_wind_vector = 'None'
    # colored noise params
    wind_colored_noise_params = 'None'

    # [plume]
    plume_model_params = 'None'

    # [camera]
    camera_view = 'None'


def set_sim_area_size(size):
    Settings.sim_area_size = str(size)
def get_sim_area_size():
    return eval(Settings.sim_area_size)

def set_odor_source_pos(pos):
    Settings.odor_source_pos = str(pos)
def get_odor_source_pos():
    return eval(Settings.odor_source_pos)

def set_dt(dt):
    Settings.dt = dt
def get_dt():
    return Settings.dt

def set_wind_model(name):
    Settings.wind_model = name
def get_wind_model():
    return Settings.wind_model

def set_wind_grid_size(gsize):
    # Notice: must be float type
    Settings.wind_grid_size = gsize
def get_wind_grid_size():
    return Settings.wind_grid_size

def set_mean_wind_vector(vector):
    Settings.mean_wind_vector = str(vector)
def get_mean_wind_vector():
    return eval(Settings.mean_wind_vector)

def set_wind_colored_noise_params(params):
    Settings.wind_colored_noise_params = str(params)
def get_wind_colored_noise_params():
    return eval(Settings.wind_colored_noise_params)

def set_plume_model_params(params):
    Settings.plume_model_params = str(params)
def get_plume_model_params():
    return eval(Settings.plume_model_params)

# camera view is tuple type: [float,float,float,array[float,float,float]]
def set_camera_view(cam):
    cam_list = [ cam[0], cam[1], cam[2], list(cam[3]) ]
    Settings.camera_view = str(cam_list)
def get_camera_view():
    cam_list = eval(Settings.camera_view)
    cam = tuple([ cam_list[0], cam_list[1], cam_list[2], np.array(cam_list[3]) ])
    return cam

def load_settings():
    # get abs path of this script
    path = sys.path[0]
    if os.path.isdir(path):
        cfgfile_path = path + '/settings.cfg'
    elif os.path.isfile(path):
        cfgfile_path = os.path.dirname(path) + '/settings.cfg'
    cp = ConfigParser.SafeConfigParser()
    cp.read(cfgfile_path)
    Settings.sim_area_size = cp.get('setup', 'sim_area_size')
    Settings.odor_source_pos = cp.get('setup', 'odor_source_pos')
    Settings.dt = cp.getfloat('setup', 'dt')
    Settings.wind_model = cp.get('wind', 'wind_model')
    Settings.wind_grid_size = cp.getfloat('wind', 'wind_grid_size')
    Settings.mean_wind_vector = cp.get('wind', 'mean_wind_vector')
    Settings.wind_colored_noise_params = cp.get('wind', 'colored_noise_params')
    Settings.plume_model_params = cp.get('plume', 'params')
    Settings.camera_view = cp.get('camera', 'camera_view')

def save_settings():
    # get abs path of this script
    path = sys.path[0]
    if os.path.isdir(path):
        cfgfile_path = path + '/settings.cfg'
    elif os.path.isfile(path):
        cfgfile_path = os.path.dirname(path) + '/settings.cfg'
    cp = ConfigParser.SafeConfigParser()
    cp.read(cfgfile_path)
    cp.set('setup', 'sim_area_size', Settings.sim_area_size)
    cp.set('setup', 'odor_source_pos', Settings.odor_source_pos)
    cp.set('setup', 'dt', str(Settings.dt))
    cp.set('wind', 'wind_model', Settings.wind_model)
    cp.set('wind', 'wind_grid_size', str(Settings.wind_grid_size))
    cp.set('wind', 'mean_wind_vector', Settings.mean_wind_vector)
    cp.set('wind', 'colored_noise_params', Settings.wind_colored_noise_params)
    cp.set('plume', 'params', Settings.plume_model_params)
    cp.set('camera', 'camera_view', Settings.camera_view)
    cp.write(open(cfgfile_path, 'w'))

