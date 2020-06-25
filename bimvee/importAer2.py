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
An example of 'aer2' format in the wild is:
    
https://github.com/VLOGroup/dvs-reconstruction/blob/master/data/sample_data.tar.bz2

It's a text file, no headers, where each event encodes one line, in the  format:
    
ts x y pol

importAer2 opens such a file (pass with filePathOrName parameter) 
and returns a dict in this format:
{'info': {},
 'data': {
         ch0: {
               dvs: {
                     'ts': np.array of np.float64 in seconds 
                     'x': np.array of np.uint16 in pixels
                     'y': np.array of np.uint16 in pixels
                     'pol': np.array of np.bool -- 1 = ON event 
                    }}}}
"""

import numpy as np
from tqdm import tqdm 

# Local imports
from .timestamps import zeroTimestampsForADataType

def inferDim(array):
    dimStops = np.array([32, 64, 128, 180, 240, 256, 260, 304, 320, 346, 480, 640, 720, 1080, 1280, 1920], dtype=np.uint16)
    idx = np.searchsorted(dimStops, np.max(array))
    try:
        return dimStops[idx]
    except IndexError:
        return np.max(array) + 1
    
def importAer2(**kwargs):
    filePathOrName = kwargs['filePathOrName']
    print('Attempting to import ' + filePathOrName + ' as aer2')
    sizeOfArray = 1024
    ts = np.zeros((sizeOfArray), dtype=np.float64)
    x = np.zeros((sizeOfArray), dtype=np.uint16)
    y = np.zeros((sizeOfArray), dtype=np.uint16)
    pol = np.zeros((sizeOfArray), dtype=np.bool)

    with open(filePathOrName, 'r') as file:
        for idx, line in enumerate(tqdm(file)):
            if idx == sizeOfArray:
                ts = np.concatenate((ts, np.zeros((sizeOfArray), dtype=np.float64)))
                x = np.concatenate((x, np.zeros((sizeOfArray), dtype=np.uint16)))
                y = np.concatenate((y, np.zeros((sizeOfArray), dtype=np.uint16)))
                pol = np.concatenate((pol, np.zeros((sizeOfArray), dtype=np.bool)))
                sizeOfArray *= 2
            lineSplit = line.split()
            ts[idx] = float(lineSplit[0])
            x[idx] = int(lineSplit[1])
            y[idx] = int(lineSplit[2])
            pol[idx] = int(lineSplit[3])
        numEvents = idx + 1
    dvsDict = {'ts': ts[:numEvents] / 1000000,
           'x': x[:numEvents],
           'y': y[:numEvents],
           'pol': pol[:numEvents],
           'dimX': inferDim(x),
           'dimY': inferDim(y)
           }

    if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)): 
        zeroTimestampsForADataType(dvsDict)
    outDict = {
    'info': {'filePathOrName':filePathOrName,
        'fileFormat': 'aer2'},
    'data': {
        'ch0': {
            'dvs': dvsDict
            }
        }
    }
    print('Done.')
    return outDict
