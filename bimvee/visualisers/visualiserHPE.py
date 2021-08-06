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
    idx = np.searchsorted(array, value) # side="left" param is the default
    if idx > 0 and ( \
            idx == len(array) or \
            math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
        return idx-1
    else:
        return idx
        
    
class VisualiserHPE(Visualiser):
    data_type = 'hpe'

    def set_data(self, data):
        self.__data = {}
        self.__data.update(data)
        
    def point_to_image(self, xS, yS, image, sktN, **kwargs):
        if xS is None:
            return image
        rad = 1 # radio for each joint point - hardcoded: could be a parameter
        if (sktN == 0):
            col = [255,0,0]
        elif (sktN == 1):
            col = [0,0,255]
        else:
            col = [0,128,0]
        try:
            for i in range(-rad,rad):
                for j in range(-rad,rad):
                    image[yS+i, xS+j, :] = col
        except IndexError:
            pass
        return image

    def get_frame(self, time, timeWindow, **kwargs):
        data = self.__data
        kwargs['startTime'] = time - timeWindow/2
        kwargs['stopTime'] = time + timeWindow/2
        kwargs['dimX'] = data['dimX']
        kwargs['dimY'] = data['dimY']
        image = getEventImageForTimeRange(data, **kwargs)
        # Post processing to get image into uint8 with correct scale
        contrast = kwargs.get('contrast', 3)
        if kwargs.get('polarised', (kwargs.get('polarized'), True)):
            image = ((image + contrast) / contrast / 2 * 255).astype(np.uint8)
            imageRGB = np.stack((image,)*3, axis=-1) 
        else:
            image = (image / contrast * 255).astype(np.uint8)
            imageRGB = np.stack((image,)*3, axis=-1)
      
        skeleton = kwargs.get('skeleton', 'None')
        if (skeleton == 'Both'):
            for skt in data['skeleton']:
                # firstIdx = np.searchsorted(data['skeleton'][skt]['ts'], time - timeWindow)
                lastIdx = np.searchsorted(data['skeleton'][skt]['ts'], time + timeWindow)
                for key in data['skeleton'][skt]:
                    if key != 'ts':
                        xS = data['skeleton'][skt][key][lastIdx,0]
                        yS = data['skeleton'][skt][key][lastIdx,1]
                        imageRGB = self.point_to_image(xS, yS, imageRGB, list(data['skeleton'].keys()).index(skt),**kwargs) 
        elif (skeleton == 'GT'):
            # firstIdx = np.searchsorted(data['skeleton']['gt']['ts'], time - timeWindow)
            lastIdx = np.searchsorted(data['skeleton']['gt']['ts'], time + timeWindow)
            for key in data['skeleton']['gt']:
                if key != 'ts':
                    xS = data['skeleton']['gt'][key][lastIdx,0]
                    yS = data['skeleton']['gt'][key][lastIdx,1]
                    imageRGB = self.point_to_image(xS, yS, imageRGB, 0,**kwargs) 
    
   
        if kwargs.get('zoom', (kwargs.get('zoom'), True)):
            zFactor = kwargs.get('zoomFactor', 5)
            halfX = int(data['dimX']/2)
            halfY = int(data['dimY']/2)
            joint = kwargs.get('jointZoom', 'handL')
            joint = kwargs.get('jointZoom', list(data['skeleton']['gt'])[0])            
            # firstIdx = np.searchsorted(data['skeleton']['gt']['ts'], time - timeWindow)
            lastIdx = np.searchsorted(data['skeleton']['gt']['ts'], time + timeWindow)
            cx = data['skeleton']['gt'][joint][lastIdx ,0]
            cy = data['skeleton']['gt'][joint][lastIdx ,1]
            imageRGB = self.point_to_image(cx, cy, imageRGB, 2,**kwargs) 
            x1 = cx - halfX//zFactor
            x2 = x1 + halfX//zFactor*2
            y1 = cy - halfY//zFactor -1
            y2 = y1 + halfY//zFactor*2
            # extract RoI
            aux = imageRGB[y1:y2, x1:x2, :]
            # zoom RoI
            aux = ndimage.zoom(aux[:, :,:], [zFactor,zFactor,1], order=1)
            # Fill with zeros non present pixels
            imageRGB = np.zeros((data['dimY'],data['dimX'],3), dtype='uint8')
            imageRGB[0:aux.shape[0], 0:aux.shape[1], :] = aux
            
        
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return imageRGB
 
    def get_dims(self):
        try:
            data = self.__data
        except AttributeError: # data hasn't been set yet
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
    
    def get_colorfmt(self):
        return 'rgb'