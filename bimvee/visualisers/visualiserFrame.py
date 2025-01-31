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

VisualiserFrame visualises frame data, using the frame in the dict nearest
to the current time.

There is an optional mode: you can add a 'tsEnd' field to the dict, an np array
of length correspinding to 'ts'. Then, the selected frame will be the last
one prior to (or exactly equal to) the current time, and will only be shown
until 'tsEnd'. Otherwise, timeWindow dictates whether the selected (nearest)
frame is shown or not.
"""

import numpy as np

# Local imports
from .visualiserBase import Visualiser

class VisualiserFrame(Visualiser):

    data_type = 'frame'

    def get_colorfmt(self):
        if len(self._data.get_data_at_time(0).shape) == 3:
            return 'rgb'
        else:
            return 'luminance'

    def get_default_image(self):
        x, y = self.get_dims()
        # Return an x,y,3 by default i.e. rgb, for safety, since in the absence of data we may not know how the texture's colorfmt is set
        return np.ones((x, y, 3), dtype=np.uint8) * 128  # TODO: Hardcoded midway (grey) value

    def get_frame(self, time, timeWindow, **kwargs):
        data = self._data
        image = data.get_data_at_time(time)
        # Allow for arbitrary post-production on image with a callback
        # TODO: as this is boilerplate, it could be pushed into pie syntax ...
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return image
