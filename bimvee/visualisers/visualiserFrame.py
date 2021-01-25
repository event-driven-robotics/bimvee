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
import math

# Local imports
from .visualiserBase import Visualiser

# A function intended to find the nearest timestamp
# adapted from https://stackoverflow.com/questions/2566412/find-nearest-value-in-numpy-array
def findNearest(array, value):
    idx = np.searchsorted(array, value) # side="left" param is the default
    if idx > 0 and ( \
            idx == len(array) or \
            math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
        return idx-1
    else:
        return idx


class VisualiserFrame(Visualiser):

    data_type = 'frame'

    def set_data(self, data):
        self.__data = {}
        self.__data.update(data)

        if self.__data['frames'][0].dtype != np.uint8:
            # Convert to uint8, converting to fullscale accross the whole dataset
            minValue = min([frame.min() for frame in self.__data['frames']])
            maxValue = max([frame.max() for frame in self.__data['frames']])
            self.__data['frames'] = [((frame-minValue)/(maxValue-minValue)*255).astype(np.uint8) for frame in data['frames']] #TODO: assuming that it starts scaled in 0-1 - could have more general approach?

    def get_colorfmt(self):
        try:
            if len(self.__data['frames'][0].shape) == 3:
                return 'rgb'
            else:
                return 'luminance'
        except: # TODO None type error?
            return 'luminance'
            
    def get_default_image(self):
        x, y = self.get_dims()
        # Return an x,y,3 by default i.e. rgb, for safety, since in the absence of data we may not know how the texture's colorfmt is set
        return np.ones((x, y, 3), dtype=np.uint8) * 128 # TODO: Hardcoded midway (grey) value

    # TODO: There can be methods which better choose the best frame, or which create a visualisation which
    # respects the time_window parameter 
    def get_frame(self, time, timeWindow, **kwargs):
        data = self.__data
        if 'tsEnd' in data:
            # Optional mode in which frames are only displayed 
            # between corresponding ts and tsEnd
            frameIdx = np.searchsorted(data['ts'], time, side='right') - 1
            if frameIdx < 0:
                image = self.get_default_image()
            elif time > data['tsEnd'][frameIdx]:
                image = self.get_default_image()
            else:                
                image = data['frames'][frameIdx]
        elif time < data['ts'][0] - timeWindow / 2 or time > data['ts'][-1] + timeWindow / 2:
            # Gone off the end of the frame data
            image = self.get_default_image()
        else:
            frameIdx = findNearest(data['ts'], time)
            image = data['frames'][frameIdx]
        # Allow for arbitrary post-production on image with a callback
        # TODO: as this is boilerplate, it could be pushed into pie syntax ...
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return image
    
    def get_dims(self):
        try:
            data = self.__data
        except AttributeError: # data hasn't been set yet
            return 1, 1
        x = data['dimX'] if 'dimX' in data else data['frames'][0].shape[1]
        y = data['dimY'] if 'dimY' in data else data['frames'][0].shape[0]
        return x, y

