# -*- coding: utf-8 -*-
"""
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
Takes a dict as imported by importAe, and for each channel, 
tries to run the appropriate general visualisation function
"""

# local imports
from .plotDvsContrast import plotDvsContrast
from .plotEventRate import plotEventRate
from .plotFrame import plotFrame
from .plotPose import plotPose
from .plotImu import plotImu
from .plotFlow import plotFlow
from .container import Container

def plot(inDict, **kwargs):
    if isinstance(inDict, Container):
        inDict = inDict.container
    
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plot(inDictInst, **kwargs)
        return
    if isinstance(inDict, dict):
        if 'info' in inDict: # Special handling for a file-level container
            kwargs['title'] = inDict['info'].get('filePathOrName', '')
            plot(inDict['data'], **kwargs)
        else:
            title = kwargs.get('title', '')
            for keyName in inDict.keys():
                if isinstance(inDict[keyName], dict):
                    if keyName == 'dvs':
                        plotDvsContrast(inDict[keyName], **kwargs)
                        plotEventRate(inDict[keyName], **kwargs)
                    elif keyName == 'frame':
                        plotFrame(inDict[keyName], **kwargs)
                    elif keyName == 'imu':
                        plotImu(inDict[keyName], **kwargs)
                    elif keyName == 'pose6q':
                        plotPose(inDict[keyName], **kwargs)
                    elif keyName == 'point3' in channel:
                        plotPose(inDict[keyName], **kwargs)
                    elif keyName == 'flow' in channel:
                        plotFlow(inDict[keyName], **kwargs)
                    else:
                        kwargs['title'] = (title + '-' + keyName).lstrip('-')
                        plot(inDict[keyName], **kwargs)
