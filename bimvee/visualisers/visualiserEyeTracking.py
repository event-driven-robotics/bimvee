# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
        Massimiliano Iacono

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
Functionality for serving images which represent the data at a certain time
(or given a certain time window).
The intended use case is to support video-like playback of data
There is a generic Visualiser class - it contains a dataType dict.
This is subclassed for each supported dataType (e.g. dvs, frame, pose etc)
Each subclass should implement basic methods: get_dims, get_frame, get_colorfmt
Implementing set_data allows the visualiser to do some preparation for visualisation
when it receives new data.

VisualiserBoundingBoxes doesn't actually create a visualisation but rather
queries the data for the bounding boxes within the time window and passes
these out for visualisation as an overlay.
"""

import numpy as np

# Local imports
from .visualiserBase import Visualiser


class VisualiserEyeTracking(Visualiser):

    data_type = 'eyeTracking'

    def get_frame(self, time, timeWindow, **kwargs):
        if self._data is None or not kwargs.get('show_eyes_gt', True):
            return None
        return self._data.get_data_at_time(time, timeWindow, **kwargs)
        

    def get_settings(self):
        settings = {'show_eyes_gt': {'type': 'boolean',
                                     'default': True
                                     },
                    'show_xy_pointcloud': {'type': 'boolean',
                                           'default': False
                                           },
                    'fixed_radius': {'type': 'boolean',
                                           'default': True
                                           },
                    'fixed_uv': {'type': 'boolean',
                                           'default': True
                                           },
                    'interpolate': {'type': 'boolean',
                                           'default': False
                                           }
                    }
        return settings
