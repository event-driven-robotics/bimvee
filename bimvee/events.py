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

from bimvee.plotDvsContrast import getEventImage
from bimvee.split import selectByBool
'''
removes events from pixels whose number of events is more than n * std above 
mean, where n is the 'threshold kwarg, with default value 3
'''
def removeHotPixels(inDict, **kwargs):
    # boilerplate to get down to dvs container
    if isinstance(inDict, list):
        for inDictSingle in inDict:
            removeHotPixels(inDictSingle, **kwargs)
        return
    if not isinstance(inDict, dict):
        return
    if 'ts' not in inDict:
        for key in inDict.keys():
            removeHotPixels(inDict[key], **kwargs)
        return
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
    