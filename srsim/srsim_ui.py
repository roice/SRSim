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
from traits.api import HasTraits, Array, Instance, Button, Enum, String
from traitsui.api import View, Item, Group, HSplit, VSplit, ButtonEditor, EnumEditor
from mayavi.core.ui.api import MlabSceneModel, SceneEditor

#############################################################################
# Module-level variables

# The grid of points on which we want to evaluate the field
#X, Y, Z = np.mgrid[0:10:100j, 0:10:100j, 0:10:100j]

#############################################################################
# ControlPanel class, srsim UI
class ControlPanel(HasTraits):
    ''' This object is the core of the traitsUI interface. Its view is the
    right panel of the application, and it hosts the method for interaction
    between the objects and the GUI.
    '''
    # Control tab

    start_stop_simulation = Button("Start/Stop simulation")
    #def _start_stop_simulation_fired(self):

    # Wind tab
    wind_field = Enum('uniform', 'external')

    # Plume tab
    plume_model = Enum('farrell', 'other')

    view = View(
            Group(
                Group(
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
                Group(
                    label = 'Robot', dock = 'tab'),
                layout = 'tabbed'),
            )

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
    # simulator running state display
    srsim_state_string = String()

    ##################################
    # The UI view to show the user
    view = View(
            HSplit(
                Item(name = 'scene',
                    editor = SceneEditor(),
                    show_label = False,
                    resizable = True),
                VSplit(
                    Item('panel', style = 'custom'),
                    Item('srsim_state_string',show_label=False,
                                        springy=True, style='custom'),
                    show_labels = False,)),
            resizable = True,
            title = 'Sniffer Robots Simulator 3D',
            height = 0.75, width = 0.75,
            )
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


##############################################################################
# Execute if running this script
if __name__ == '__main__':
    #app = ControlPanel()
    app = MainWindow()
    # open a dialog
    app.configure_traits()
