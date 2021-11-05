# -*- coding: utf-8 -*-
"""
Copyright (C) 2021 Event-driven Perception for Robotics
Authors: Franco Di Pietro 
        Sim Bamford
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
from scipy import ndimage
import math
# Local imports
from ..plotDvsContrast import getEventImageForTimeRange
from .visualiserBase import Visualiser


def findNearest(array, value):
    idx = np.searchsorted(array, value)  # side="left" param is the default
    if idx > 0 and (idx == len(array) or math.fabs(value - array[idx - 1]) < math.fabs(value - array[idx])):
        return idx - 1
    else:
        return idx


class VisualiserSkeleton(Visualiser):
    data_type = 'skeleton'

    def set_data(self, data):
        self.__data = {}
        self.__data.update(data)

    def point_to_image(self, xS, yS, image, sktN, **kwargs):
        if xS is None:
            return image
        rad = 1  # radio for each joint point - hardcoded: could be a parameter
        if (sktN == 0):
            col = [255, 0, 0]
        elif (sktN == 1):
            col = [0, 0, 255]
        else:
            col = [0, 128, 0]
        try:
            for i in range(-rad, rad):
                for j in range(-rad, rad):
                    image[yS + i, xS + j, :] = col
        except IndexError:
            pass
        return image

    def get_frame(self, time, timeWindow, **kwargs):
        if not kwargs.get('show_skeleton'):
            return None
        data = self.__data
        kwargs['startTime'] = time - timeWindow / 2
        kwargs['stopTime'] = time + timeWindow / 2

        # firstIdx = np.searchsorted(data[skt]['ts'], time - timeWindow)
        lastIdx = np.searchsorted(data['ts'], time + timeWindow) - 1
        outData = {}
        for key in data:
            if key != 'ts' and key != 'tsOffset':
                outData[key] = [data[key][lastIdx, 0],
                                data[key][lastIdx, 1]]

        return outData

    def get_settings(self):
        # settings[data_type] = {'type': 'boolean',
        #                                     'default': True
        #                                     }
        settings = {'show_skeleton': {'type': 'boolean',
                                      'default': True},
                    'show_labels': {'type': 'boolean',
                                    'default': False
                                    }
                    }

        # ,
        # 'zoom': {'type': 'boolean',
        #          'default': False
        #          },
        # 'zoomFactor': {'type': 'range',
        #                'default': 9,
        #                'min': 4,
        #                'max': 15,
        #                'step': 1
        #                },
        # 'jointZoom': {'type': 'value_list',
        #               'default': list(
        #                   self.__data[
        #                       'gt'].keys())[0],
        #               'values': self.__data[
        #                   'gt'].keys()
        #               }
        # }

        return settings
