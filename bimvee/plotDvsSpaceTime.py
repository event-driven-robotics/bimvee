# -*- coding: utf-8 -*-
'''
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
plotDvsSpaceTime takes 'inDict' - a dict containing an imported ae file, 
as created by importAe, crops it spatially and temporally according to any kwargs
such as maxTim etc
and creates a point cloud viewer containing 

...
'''

import numpy as np
import pptk

from .split import cropSpaceTime

def plotDvsSpaceTime(inDicts, **kwargs):
    if isinstance(inDicts, list):
        for inDict in inDicts:
            plotDvsSpaceTime(inDict, **kwargs)
        return
    else:
        inDict = inDicts
    if not isinstance(inDict, dict):
        return
    if 'ts' not in inDict:
        #title = kwargs.pop('title', '')
        if 'info' in inDict and isinstance(inDict, dict):
            fileName = inDict['info'].get('filePathOrName')
            if fileName is not None:
                print('plotDvsContrast was called for file ' + fileName)
                #title = (title + ' ' + fileName).lstrip()
        for key in inDict.keys():
        #    kwargs['title'] = (title + ' ' + key).lstrip()
            plotDvsSpaceTime(inDict[key], **kwargs)
        return
    # From this point onwards, it's a data-type container
    if 'pol' not in inDict:
        return
    # From this point onwards, it's a dvs container        

    inDict = cropSpaceTime(inDict, **kwargs)
    
    # scale x and y to match time range
    timeRange = inDict['ts'][-1] - inDict['ts'][0]
    spatialDim = max(max(inDict['x']), max(inDict['y']))
    scalingFactor = timeRange / spatialDim
    events = np.concatenate((inDict['x'][:, np.newaxis] * scalingFactor, 
                             inDict['y'][:, np.newaxis] * scalingFactor, 
                             inDict['ts'][:, np.newaxis]), axis=1)
    pptkViewer = pptk.viewer(events) 
    return pptkViewer
