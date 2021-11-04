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

VisualisePoint3 shows a basic point cloud containing every point in the dict
with a timestamp within the time window. 
The world (everywhere the points reach in the whole data) is compressed into
a rectangle which is then viewed from the outside as if the world reference
frame were aligned with a standard camera reference frame,
so z increases away from the viewer, x increases to the right,
and y in a downwards direction.
The user has a choice to apply perspective (scale decreases away from the 
viewer) or not.
There are also pitch and yaw controls - this alters the basic viewpoint of
the viewer described above.
"""

import numpy as np

# Local imports
from .visualiserBase import Visualiser


class VisualiserPoint3(Visualiser):
    renderX = 300  # TODO Hardcoded
    renderY = 300
    labels = None
    data_type = 'point3'

    def __init__(self, data):
        self.set_data(data)
        self.smallestRenderDim = min(self.renderX, self.renderY)

    '''
    Offset and scale the pose translations so that they all fit into the volume:
        x [-0.5:0.5]
        y[-0.5:0.5]
        z[1:2]
    '''

    def set_data(self, data):
        # scale and offset point data so that it remains proportional 
        # but stays in the range 0-1 for all dimensions
        pointX = data['point'][:, 0]
        pointY = data['point'][:, 1]
        pointZ = data['point'][:, 2]
        minX = np.min(pointX)
        maxX = np.max(pointX)
        minY = np.min(pointY)
        maxY = np.max(pointY)
        minZ = np.min(pointZ)
        maxZ = np.max(pointZ)
        centreX = (minX + maxX) / 2
        centreY = (minY + maxY) / 2
        centreZ = (minZ + maxZ) / 2
        largestDim = max(maxX - minX, maxY - minY, maxZ - minZ)
        if largestDim == 0:
            largestDim = 1

        pointScaled = np.empty_like(data['point'])
        pointScaled[:, 0] = pointX - centreX
        pointScaled[:, 1] = pointY - centreY
        pointScaled[:, 2] = pointZ - centreZ
        pointScaled = pointScaled / largestDim + 0.5
        internalData = {'ts': data['ts'],
                        'point': pointScaled
                        }
        self.__data = internalData

    def project3dTo2d(self, x=0, y=0, z=0, **kwargs):
        smallestRenderDim = kwargs.get('smallestRenderDim', 1)
        windowFill = kwargs.get('windowFill', 0.9)
        if kwargs.get('perspective', True):
            # Move z out by 1, so that the data is between 1 and 2 distant in z
            # x and y are in range 0-1, so they get shifted to be centred around 0 during this operation
            x = (x - 0.5) / (z + 1) + 0.5
            y = (y - 0.5) / (z + 1) + 0.5
        x = (x * windowFill + (1 - windowFill) / 2) * smallestRenderDim
        y = (y * windowFill + (1 - windowFill) / 2) * smallestRenderDim
        x = x.astype(int)
        y = y.astype(int)
        return x, y

    def point_to_image(self, point, image, **kwargs):
        if point is None:
            return image
        # Unpack
        pointX = point[0]
        pointY = point[1]
        pointZ = point[2]
        # Project the location
        projX, projY = self.project3dTo2d(x=pointX, y=pointY, z=pointZ,
                                          smallestRenderDim=self.smallestRenderDim, **kwargs)
        try:
            image[projY, projX, :] = 255
        except IndexError:  # perspective or other projection issues cause out of bounds? ignore
            pass
        return image

    def get_frame(self, time, timeWindow, **kwargs):
        data = self.__data
        if data is None:
            print('Warning: data is not set')
            return np.zeros((1, 1), dtype=np.uint8)  # This should not happen
        image = np.zeros((self.renderY, self.renderX, 3), dtype=np.uint8)
        # Put a grey box around the edge of the image
        image[0, :, :] = 128
        image[-1, :, :] = 128
        image[:, 0, :] = 128
        image[:, -1, :] = 128
        # Put a grey crosshair in the centre of the image
        rY = self.renderY
        rX = self.renderY
        chp = 20  # Cross Hair Proportion for following expression
        image[int(rY / 2 - rY / chp): int(rY / 2 + rY / chp), int(rX / 2), :] = 128
        image[int(rY / 2), int(rX / 2 - rX / chp): int(rX / 2 + rX / chp), :] = 128
        firstIdx = np.searchsorted(data['ts'], time - timeWindow)
        lastIdx = np.searchsorted(data['ts'], time + timeWindow)
        points = data['point'][firstIdx:lastIdx, :]
        # Use yaw and pitch sliders to transform points
        yaw = -kwargs.get('yaw', 0) / 180 * np.pi
        pitch = kwargs.get('pitch', 0) / 180 * np.pi
        roll = 0
        cosA = np.cos(roll)
        cosB = np.cos(yaw)
        cosC = np.cos(pitch)
        sinA = np.sin(roll)
        sinB = np.sin(yaw)
        sinC = np.sin(pitch)
        rotMat = np.array([[cosA * cosB, cosA * sinB * sinC - sinA * cosC, cosA * sinB * cosC + sinA * sinC],
                           [sinA * cosB, sinA * sinB * sinC + cosA * cosC, sinA * sinB * cosC - cosA * sinC],
                           [-sinB, cosB * sinC, cosB * cosC]],
                          dtype=np.float64)
        points = points - 0.5
        points = np.matmul(rotMat, points.transpose()).transpose()
        points = points + 0.5

        for row in points:
            image = self.point_to_image(row, image, **kwargs)

            # Allow for arbitrary post-production on image with a callback
        # TODO: as this is boilerplate, it could be pushed into pie syntax ...
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return image

    def get_dims(self):
        return self.renderX, self.renderY

    def get_colorfmt(self):
        return 'rgb'

    def get_settings(self):
        settings = {'perspective': {'type': 'boolean',
                                    'default': True
                                    },
                    'yaw': {'type': 'range',
                            'default': 0,
                            'min': -90,
                            'max': 90,
                            'step': 1
                            },
                    'pitch': {'type': 'range',
                              'default': 0,
                              'min': -90,
                              'max': 90,
                              'step': 1
                              }}

        return settings
