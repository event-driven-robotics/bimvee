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

VisualiseDvs creates an event frame centred at time and of size timeWindow.
The user may set the amount of contrast (concurrent events in a pixel to reach
full colour) and whether to render polarity (in which case ON and OFF events
are counted against each other).
"""

import numpy as np

# Local imports
from ..plotDvsContrast import getEventImageForTimeRange
from .visualiserBase import Visualiser


class VisualiserDvs(Visualiser):
    data_type = 'dvs'
    coloured = False

    def set_data(self, data):
        self.__data = {}
        self.__data.update(data)

    # TODO: There can be methods which better choose the best frame, or which create a visualisation which
    # respects the time_window parameter 
    def get_frame(self, time, timeWindow, **kwargs):
        self.coloured = kwargs['image_type'] == 'coloured'
        data = self.__data
        kwargs['startTime'] = time - timeWindow / 2
        kwargs['stopTime'] = time + timeWindow / 2
        kwargs['dimX'] = data['dimX']
        kwargs['dimY'] = data['dimY']
        image = getEventImageForTimeRange(data, **kwargs)
        # Post processing to get image into uint8 with correct scale
        contrast = kwargs.get('contrast', 3)
        if kwargs.get('image_type') == 'coloured':
            pass
        elif kwargs.get('image_type') == 'count' or kwargs.get('image_type') == 'binary':
            image = ((image + contrast) / contrast / 2 * 255).astype(np.uint8)
        else:
            image = (image / contrast * 255).astype(np.uint8)
        # Allow for arbitrary post-production on image with a callback
        # TODO: as this is boilerplate, it could be pushed into pie syntax ...
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return image

    def get_dims(self):
        try:
            data = self.__data
        except AttributeError:  # data hasn't been set yet
            return 1, 1
        if 'dimX' in data:
            x = data['dimX']
        else:
            x = np.max(data['x']) + 1
            data['dimX'] = x
        if 'dimY' in data:
            y = data['dimY']
        else:
            y = np.max(data['y']) + 1
            data['dimY'] = y
        return x, y

    def get_settings(self):
        settings = {'image_type': {'type': 'value_list',
                                   'default': 'binary',
                                   'values': ['count', 'binary', 'not_polarized', 'time_image', 'coloured']
                                   },
                    'contrast': {'type': 'range',
                                 'default': 3,
                                 'min': 1,
                                 'max': 20,
                                 'step': 1
                                 },
                    'pol_to_show': {'type': 'value_list',
                                    'default': 'Both',
                                    'values': ['Pos', 'Neg', 'Both']
                                    }}
        return settings

    def get_colorfmt(self):
        return 'rgb' if self.coloured else 'luminance'