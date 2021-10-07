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

VisualiserPose6q is for visualising pose6q class.
An orientation frame (perpendicular unit vectors of rgb) are generated for each
bodyId (It can handle multiple bodyIds in the same dataDict)
The world (everywhere the trajectories reach in the data) is compressed into
a rectangle which is then viewed from the outside as if the world reference
frame were aligned with a standard camera reference frame,
so z increases away from the viewer, x increases to the right,
and y in a downwards direction.
The user has a choice to apply perspective (scale decreases away from the 
viewer) or not.
There is also a choice of interpolation - if chosen, the pose drawn for a given
time is linearly interpolated between the two nearest poses.
"""

import numpy as np
import math
import cv2

# Local imports
from ..geometry import quat2RotM, slerp
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
    xVec = np.expand_dims(np.array([unitLength, 0, 0, 1]), axis=1)
    yVec = np.expand_dims(np.array([0, unitLength, 0, 1]), axis=1)
    zVec = np.expand_dims(np.array([0, 0, unitLength, 1]), axis=1)
    allVecs = np.concatenate((xVec, yVec, zVec), axis=1)
    return rotM.dot(allVecs)


class VisualiserPose6q(Visualiser):
    renderX = 200  # TODO Hardcoded
    renderY = 200
    labels = None
    data_type = 'pose6q'

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
                        'point': pointScaled,
                        'rotation': data['rotation']}
        # Having scaled data for all poses, split out the poses for different bodies, if applicable
        if 'bodyId' in data:
            # split pose data by label, for ease of reference during rendering
            internalData['bodyId'] = data['bodyId']
            self.__data = splitByLabel(internalData, 'bodyId', outList=False)
            self.labels = np.unique(data['bodyId'])
        else:
            self.__data = {'': internalData}

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
            return image, None
        # Unpack
        pointX = point[0]
        pointY = point[1]
        pointZ = point[2]
        # Project the location
        kwargs['smallestRenderDim'] = self.smallestRenderDim
        projX, projY = self.project3dTo2d(x=pointX, y=pointY, z=pointZ, **kwargs)
        if rotation is None:
            # Just put a white dot where the point should be
            image[projY, projX, :] = 255
        else:
            rotMats = quat2RotM(rotation)
            rotatedUnitVectors = rotateUnitVectors(rotMats, unitLength=0.25)
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
        return image, (projX, projY)

    def get_frame(self, time, timeWindow, **kwargs):
        allData = self.__data
        if allData is None:
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
        for dataKey in allData.keys():
            data = allData[dataKey]
            # Note, for the following, this follows library function pose6qInterp, but is broken out here, because of slight behavioural differences.
            idxPre = np.searchsorted(data['ts'], time, side='right') - 1
            timePre = data['ts'][idxPre]
            if timePre == time:
                # In this edge-case of desired time == timestamp, there is no need 
                # to interpolate 
                point = data['point'][idxPre, :]
                rotation = data['rotation'][idxPre, :]
            elif idxPre < 0 or (idxPre >= len(data['ts']) - 1):
                # In this edge-case of the time at the beginning or end, 
                # don't show any pose
                point = None
                rotation = None
            else:
                if kwargs.get('interpolate', True):
                    timePost = data['ts'][idxPre + 1]
                    qPre = data['rotation'][idxPre, :]
                    qPost = data['rotation'][idxPre + 1, :]
                    timeRel = (time - timePre) / (timePost - timePre)
                    rotation = slerp(qPre, qPost, timeRel)
                    locPre = data['point'][idxPre, :]
                    locPost = data['point'][idxPre + 1, :]
                    point = locPre * (1 - timeRel) + locPost * timeRel
                    timeDist = min(time - timePre, timePost - time)
                    if timeDist > timeWindow / 2:
                        # Warn the viewer that this interpolation is 
                        # based on data beyond the timeWindow
                        image[:30, :30, 0] = 255  # TODO: Hardcoded
                else:  # No interpolation, so just choose the sample which is nearest in time
                    poseIdx = findNearest(data['ts'], time)
                    point = data['point'][poseIdx, :]
                    rotation = data['rotation'][poseIdx, :]
            image, coords = self.project_pose(point, rotation, image, **kwargs)
            if kwargs.get('label_multiple_bodies', True) and coords is not None:
                cv2.putText(
                    image,  # numpy array on which text is written
                    dataKey,  # text
                    coords,  # position at which writing has to start
                    cv2.FONT_HERSHEY_SIMPLEX,  # font family
                    0.2,  # font size
                    (255, 255, 255, 255),  # font color
                    1)  # font stroke

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
        settings = {'interpolate': {'type': 'boolean',
                                    'default': True
                                    },
                    'perspective': {'type': 'boolean',
                                    'default': True
                                    }}
        return settings
