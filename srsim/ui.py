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
from traits.api import HasTraits, Instance, Property, Enum
from traitsui.api import View, Item, HSplit, VSplit, InstanceEditor
from tvtk.pyface.scene_editor import SceneEditor
from mayavi.core.ui.engine_view import EngineView
from mayavi.tools.mlab_scene_model import MlabSceneModel

#############################################################################
# Module-level variables

# The grid of points on which we want to evaluate the field
#X, Y, Z = np.mgrid[0:10:100j, 0:10:100j, 0:10:100j]

#############################################################################
# ui class
class srsim_ui(HasTraits):

    # The scene model.
    scene = Instance(MlabSceneModel, ())

    # The mayavi engine view.
    engine_view = Instance(EngineView)

    # The current selection in the engine tree view.
    current_selection = Property



##############################################################################
# Execute if running this script
if __name__ == '__main__':
    app = srsim_ui()
    app.configure_traits()
