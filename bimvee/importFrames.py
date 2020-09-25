# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
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

importFrames imports a folder full of (timestamped) frames saved as image files.  
It might make sense to generalise this using e.g. https://github.com/soft-matter/pims
but on day one this imports frames in the format exported by:
https://github.com/uzh-rpg/rpg_e2vid
i.e. a folder full of png, also containing a 'timestamps.txt' file, with a timestamp
in seconds written on each line. 

Returns a dict:
{'info': {'filePathOrName': str},
 'data': {
     channel0: {
         frame: {
             "ts": numpy array of float - seconds
             "frames": a list of numpy arrays where dim 0 = y (increasing downwards)

"""

#%%

import re
import numpy as np
import os
from tqdm import tqdm
import struct
import imageio

# local imports
from .timestamps import zeroTimestampsForAChannel, rezeroTimestampsForAnImportedDict

def getOrInsertDefault(inDict, arg, default):
    # get an arg from a dict.
    # If the the dict doesn't contain the arg, return the default, 
    # and also insert the default into the dict
    value = inDict.get(arg, default)
    if value == default:
        inDict[arg] = default
    return value

def importFrames(**kwargs):
    path = getOrInsertDefault(kwargs, 'filePathOrName', '.')
    print('importFrames trying path: ' + path)
    if not os.path.exists(path):
        raise FileNotFoundError("path not found.")
    if not os.path.isdir(path):
        raise FileNotFoundError("path is not a directory.")
    files = sorted(os.listdir(path))
    # TODO: trusting the os to sort the files may not work
    frames = []
    for file in tqdm(files):
        filePathAndName = os.path.join(path, file) 
        if file == 'timestamps.txt': # todo: is there a more general form?
            ts = np.loadtxt(filePathAndName)
        elif os.path.isfile(filePathAndName):
            frames.append(imageio.imread(filePathAndName))
    channelDict = {'frame': 
                       {'ts': ts,
                        'frames': frames}}
    if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)):
        zeroTimestampsForAChannel(channelDict)
    importedDict = {
        'info': kwargs,
        'data': {'ch0': channelDict}
        }
    importedDict['info']['fileFormat'] = 'imagefolder'
    if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)):
        rezeroTimestampsForAnImportedDict(importedDict)
    return importedDict
        
    
    