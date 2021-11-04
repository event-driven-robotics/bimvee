# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Author: Sim Bamford

This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
Basic manipulations specific to event streams
"""

import numpy as np
from tqdm import trange
import cv2

from bimvee.plotDvsContrast import getEventImage
from bimvee.split import selectByBool

'''
removes events from pixels whose number of events is more than n * std above 
mean, where n is the 'threshold kwarg, with default value 3
'''
def removeHotPixels(inDict, **kwargs):
    # boilerplate to get down to dvs container
    if isinstance(inDict, list):
        return [removeHotPixels(inDictSingle, **kwargs)
                for inDictSingle in inDict]
    if not isinstance(inDict, dict):
        return inDict
    if 'ts' not in inDict:
        outDict = {}
        for key in inDict.keys():
            outDict[key] = removeHotPixels(inDict[key], **kwargs)
        return outDict
    # From this point onwards, it's a data-type container
    if 'pol' not in inDict:
        return
    # From this point onwards, it's a dvs container
    events = inDict
    eventImage = getEventImage(events, contrast=np.inf, polarised=False)
    contrast1d = eventImage.flatten()
    mean = np.mean(contrast1d)
    std = np.std(contrast1d)
    threshold = mean + kwargs.get('threshold', 3) * std
    (y, x) = np.where(eventImage > threshold)
    dimY = kwargs.get('dimY', events.get('dimY', events['y'].max() + 1))
    #dimX = kwargs.get('dimX', events.get('dimX', events['x'].max()))
    addrsToRemove = x * dimY + y
    eventAddrs = events['x'] * dimY + events['y']
    toKeep = np.logical_not(np.isin(eventAddrs, addrsToRemove))
    return selectByBool(events, toKeep)

''' 
Iterates event by event, using a prevTs array to keep track of each pixel. 
From a brief trial, dissecting into different arrays, 
doing the filter and then merging again is quite a lot slower than this. 
'''
def refractoryPeriod(inDict, refractoryPeriod=0.001, **kwargs):
    # boilerplate to get down to dvs container
    if isinstance(inDict, list):
        return [refractoryPeriod(inDictSingle, **kwargs)
                for inDictSingle in inDict]
    if not isinstance(inDict, dict):
        return inDict
    if 'ts' not in inDict:
        outDict = {}
        for key in inDict.keys():
            outDict[key] = refractoryPeriod(inDict[key], **kwargs)
        return outDict
    # From this point onwards, it's a data-type container
    if 'pol' not in inDict:
        return
    # From this point onwards, it's a dvs container
    events = inDict
    ts = events['ts']
    x = events['y']
    y = events['x']
    numEvents = len(ts)
    maxX = x.max()
    maxY = y.max()
    prevTs = np.zeros((maxY+1, maxX+1))
    toKeep = np.ones((numEvents), dtype=np.bool)
    for idx in trange(numEvents, leave=True, position=0):
        if ts[idx] >= prevTs[y[idx], x[idx]] + refractoryPeriod: 
            prevTs[y[idx], x[idx]] = ts[idx]
        else:
            toKeep[idx] = False
    outDict = selectByBool(inDict, toKeep)
    return outDict

'''
if dict does not already have dimX and dimY fields, then add them, using the
maximum value in each dimension.
NOTE: This is a hap-hazard method which relies on pixels actually producing 
events; only to be used if camera dimension is not otherwise available.
'''
def findDims(inDict):
    if 'dimX' in inDict and 'dimY' in inDict:
        return inDict
    outDict = inDict.copy()
    if 'dimX' not in inDict:
        outDict['dimX'] = np.max(inDict['x']) + 1
    if 'dimY' not in inDict:
        outDict['dimY'] = np.max(inDict['y']) + 1
    return outDict

'''
Converts the underlying representation of address-events to a single ndarray
n x 3, where the first col is x, the second col is y, and the third col is all 
ones; the dtype is np.float64. The field is called 'xyh'.
x and y fields become 1d views of the appropriate rows.
'''
def convertToHomogeneousCoords(inDict):
    outDict = inDict.copy()
    outDict['xyh'] = np.concatenate((
        inDict['x'][:, np.newaxis].astype(np.float64),
        inDict['y'][:, np.newaxis].astype(np.float64),
        np.ones((inDict['x'].shape[0], 1), dtype=np.float64)
        ), axis=1)
    outDict['x'] = outDict['xyh'][:, 0]
    outDict['y'] = outDict['xyh'][:, 1]
    return outDict

'''
Uses opencv functions to create an undistortion map and undistort events.
k is the intrinsic matrix; d is the distortion coefficients.
Returns a new container with the events undistorted.
By default, events get turned into float64; keep them as int16 with kwarg 'asInt'=True
By default, it uses the same intrinsic matrix in the remapping - change this
by passing in the kwarg 'kNew'
'''
def undistortEvents(inDict, k, d, **kwargs):
    inDict = findDims(inDict)
    kNew = kwargs.get('kNew', k)
    yGrid, xGrid = np.meshgrid(range(inDict['dimY']), range(inDict['dimX']))
    xGrid = xGrid.reshape(-1).astype(np.float32)
    yGrid = yGrid.reshape(-1).astype(np.float32)
    xyGrid = np.concatenate((xGrid[:, np.newaxis], yGrid[:, np.newaxis]), axis=1)
    undistortedPoints = cv2.undistortPoints(xyGrid, k, d, None, kNew)
    undistortionMap = undistortedPoints.reshape(inDict['dimX'], inDict['dimY'], 2)
    undistortionMap = np.swapaxes(undistortionMap, 0, 1)
    xy = undistortionMap[inDict['y'], inDict['x'], :]
    if kwargs.get('asInt', False):
        xy = np.round(xy).astype('np.int16')
    outDict = inDict.copy()
    outDict['x'] = xy[:, 0]
    outDict['y'] = xy[:, 1]
    return outDict