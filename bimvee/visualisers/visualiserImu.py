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

VisualiserImu is for visualising imu data. A single 'cursor' is plotted, with
orthogonal unit lines pointing in x (red) y (green) and z (blue) directions.
The position of the cursor w.r.t. the centre of the visualisation indicates
linear acceleration; the orientation of the cursor indicates angular velocity.
The gain of the orientation can be altered using the kwarg 'rotation_scale' -
this interprets an input on a scale from 0 to 100 (default 50) as a multiplier
exponentially distributed in the range 0.01 through 1 (default) to 100.
The 'smoothing' kwarg (default = 0) increases the number of samples either side
which are averaged to produce the sample visualised. The input, expected to be
in the range 0 - 100, is interpretted as an exponentially distributed sample number
reaching around 117 samples each side, i.e. 235 samples in total.
"""

import numpy as np
import math
from scipy import linalg

# Local imports
from ..split import splitByLabel
from .visualiserBase import Visualiser


# A function intended to find the nearest timestamp
# adapted from https://stackoverflow.com/questions/2566412/find-nearest-value-in-numpy-array
def findNearest(array, value):
    idx = np.searchsorted(array, value)  # side="left" param is the default
    if idx > 0 and ( \
                    idx == len(array) or \
                    math.fabs(value - array[idx - 1]) < math.fabs(value - array[idx])):
        return idx - 1
    else:
        return idx


def angularVelocityToRotMat(angV):
    try:
        return linalg.expm(np.array(
            [[0, -angV[2], angV[1]],
             [angV[2], 0, -angV[0]],
             [-angV[1], angV[0], 0]]))
    except OverflowError:
        print('rotation conversion overflow')
        return np.eye(3)


# %% Two helper functions for pose visualiser

# adapted from https://stackoverflow.com/questions/50387606/python-draw-line-between-two-coordinates-in-a-matrix
def draw_line(mat, x0, y0, x1, y1):
    if (x0, y0) == (x1, y1):
        if x0 >= 0 and x0 < mat.shape[1] and y0 >= 0 and y0 < mat.shape[0]:
            mat[x0, y0] = 255
        return
    # Swap axes if Y slope is smaller than X slope
    transpose = abs(x1 - x0) < abs(y1 - y0)
    if transpose:
        mat = mat.T
        x0, y0, x1, y1 = y0, x0, y1, x1
    # Swap line direction to go left-to-right if necessary
    if x0 > x1:
        x0, y0, x1, y1 = x1, y1, x0, y0
    # Compute intermediate coordinates using line equation
    x = np.arange(x0, x1 + 1)
    y = np.round(((y1 - y0) / (x1 - x0)) * (x - x0) + y0).astype(x.dtype)
    # Write intermediate coordinates
    toKeep = np.logical_and(x >= 0,
                            np.logical_and(x < mat.shape[1],
                                           np.logical_and(y >= 0, y < mat.shape[0])))
    mat[y[toKeep], x[toKeep]] = 255


def rotateUnitVectors(rotM, unitLength=1.0):
    xVec = np.expand_dims(np.array([unitLength, 0, 0]), axis=1)
    yVec = np.expand_dims(np.array([0, unitLength, 0]), axis=1)
    zVec = np.expand_dims(np.array([0, 0, unitLength]), axis=1)
    allVecs = np.concatenate((xVec, yVec, zVec), axis=1)
    return rotM.dot(allVecs)


class VisualiserImu(Visualiser):
    renderX = 200  # TODO Hardcoded
    renderY = 200
    labels = None
    data_type = 'imu'

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
        minX = np.min(data['acc'][:, 0])
        maxX = np.max(data['acc'][:, 0])
        minY = np.min(data['acc'][:, 1])
        maxY = np.max(data['acc'][:, 1])
        minZ = np.min(data['acc'][:, 2])
        maxZ = np.max(data['acc'][:, 2])
        largestDim = max(maxX, -minX, maxY, -minY, maxZ, -minZ)
        if largestDim == 0:
            largestDim = 1

        pointScaled = data['acc'] / largestDim / 2 + 0.5
        self.__data = {'ts': data['ts'],
                       'point': pointScaled,
                       'rotation': data['angV']}

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

    def project_pose(self, point, rotation, image, **kwargs):
        if point is None:
            return image
        # Unpack
        pointX = point[0]
        pointY = point[1]
        pointZ = point[2]
        # Project the location
        kwargs['smallestRenderDim'] = self.smallestRenderDim
        rotationScale = kwargs.get('rotation_scale', 50)
        rotationScale = 10 ** ((rotationScale - 50) / 25)
        projX, projY = self.project3dTo2d(x=pointX, y=pointY, z=pointZ, **kwargs)
        rotMat = angularVelocityToRotMat(rotation * rotationScale)
        rotatedUnitVectors = rotateUnitVectors(rotMat, unitLength=0.25)
        # aVectorCoordB means the Bth coordinate of unit vector originally directed towards a
        xVectorCoordX = pointX + rotatedUnitVectors[0, 0]
        xVectorCoordY = pointY + rotatedUnitVectors[1, 0]
        xVectorCoordZ = pointZ + rotatedUnitVectors[2, 0]
        yVectorCoordX = pointX + rotatedUnitVectors[0, 1]
        yVectorCoordY = pointY + rotatedUnitVectors[1, 1]
        yVectorCoordZ = pointZ + rotatedUnitVectors[2, 1]
        zVectorCoordX = pointX + rotatedUnitVectors[0, 2]
        zVectorCoordY = pointY + rotatedUnitVectors[1, 2]
        zVectorCoordZ = pointZ + rotatedUnitVectors[2, 2]
        xVectorProjX, xVectorProjY = self.project3dTo2d(x=xVectorCoordX, y=xVectorCoordY, z=xVectorCoordZ, **kwargs)
        yVectorProjX, yVectorProjY = self.project3dTo2d(x=yVectorCoordX, y=yVectorCoordY, z=yVectorCoordZ, **kwargs)
        zVectorProjX, zVectorProjY = self.project3dTo2d(x=zVectorCoordX, y=zVectorCoordY, z=zVectorCoordZ, **kwargs)
        draw_line(image[:, :, 0], projX, projY, xVectorProjX, xVectorProjY)
        draw_line(image[:, :, 1], projX, projY, yVectorProjX, yVectorProjY)
        draw_line(image[:, :, 2], projX, projY, zVectorProjX, zVectorProjY)
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
        rX = self.renderX
        chp = 20  # Cross Hair Proportion for following expression
        image[int(rY / 2 - rY / chp): int(rY / 2 + rY / chp), int(rX / 2), :] = 128
        image[int(rY / 2), int(rX / 2 - rX / chp): int(rX / 2 + rX / chp), :] = 128
        # Note, for the following, this follows library function pose6qInterp, but is broken out here, because of slight behavioural differences.
        idxPre = np.searchsorted(data['ts'], time, side='right') - 1
        timePre = data['ts'][idxPre]
        if idxPre < 0 or (idxPre >= len(data['ts']) - 1):
            # In this edge-case of the time at the beginning or end,
            # don't show any pose
            return image
        elif timePre == time:
            # In this edge-case of desired time == timestamp, there is no need
            # to interpolate
            poseIdx = idxPre
        else:
            poseIdx = findNearest(data['ts'], time)

        smoothing = int(kwargs.get('smoothing', 0))
        if smoothing:
            smoothing = int(1.1 ** smoothing / 2)
            firstIdx = max(0, poseIdx - smoothing)
            lastIdx = min(len(data['ts']), poseIdx + smoothing + 1)
            point = np.mean(data['point'][firstIdx:lastIdx, :], axis=0)
            rotation = np.mean(data['rotation'][firstIdx:lastIdx, :], axis=0)
        else:
            point = data['point'][poseIdx, :]
            rotation = data['rotation'][poseIdx, :]
        image = self.project_pose(point, rotation, image, **kwargs)

        return image

    def get_dims(self):
        return self.renderX, self.renderY

    def get_colorfmt(self):
        return 'rgb'

    def get_settings(self):
        settings = {'perspective': {'type': 'boolean',
                                    'default': True
                                    },
                    'rotation_scale': {'type': 'range',
                                       'default': 50,
                                       'min': 0,
                                       'max': 100,
                                       'step': 1
                                       },
                    'smoothing': {'type': 'range',
                                  'default': 0,
                                  'min': 0,
                                  'max': 100,
                                  'step': 1
                                  }}
        return settings
