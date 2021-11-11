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
importAe is a function for importing timestamped address-event data, given a path 
(defaulting to the current directory, intended for the yarp format) or a file.  
If the file format is not stated, there is an attempt to determine this from the file.
Then a sub-function is called, specialised for importing the data contained 
into the workspace. Depending on the format, additional data may also be imported,
including frame data, imu samples, skin data, 2d or 3d coords etc etc.
  
The output is a dictionary containing:
    - info
    - data
The exact contents varies according to the file type import but in general:
    info: this is a dict which starts life as the kwargs passed in, and is 
    augmented and modified by the actual contents of the file. It may include 
    any informational fields in file headers. Minimally, it will contain:
        - filePathAndName
        - fileFormat
    data: this is a list of dicts, one for each sensor or "channel" which has 
    been imported. Bear in mind that sub-functions may optionally split or join 
    channels. Within each dict, there is a field for each type of data contained.
    A file for example may contain data from a several sensors, but a single sensor 
    may produce polarity events ("pol"), aps samples ("aps"), imu samples etc.
    Within each of these fields, there is a dict, generally containing fields for 
    each data column, so in the case of pol events, there are 4-5 fields:
        - ts
        - x
        - y 
        - pol
        - optionally ch (channel)
        each containing a numpy array of the appropriate type for the data 
        contained, where all these arrays will be of the same length.

Aim is to support:

YARP .log - ATIS Gen1 - 24 bit - includes, IMU, Vicon, (also SKIN?)
rpg_dvs_ros - DVS/DAVIS
Maybe others? 
jAER / cAER .aedat (v1/2/3) DVS / DAVIS / Cochlea?
Samsung Gen3 VGA?
Celex ...???
"""

#%%

import os

# local imports
from .importIitNumpy import importIitNumpy
from .importIitYarp import importIitYarp
from .importRpgDvsRos import importRpgDvsRos
from .importSecDvs import importSecDvs
from .importAer2 import importAer2
from .importFrames import importFrames
from .timestamps import rezeroTimestampsForImportedDicts
from .importHdf5 import importHdf5
from .importProph import importProph

def getOrInsertDefault(inDict, arg, default):
    # get an arg from a dict.
    # If the the dict doesn't contain the arg, return the default, 
    # and also insert the default into the dict
    value = inDict.get(arg, default)
    if value == default:
        inDict[arg] = default
    return value


def importAe(**kwargs):
    print(kwargs)
    filePathOrName = getOrInsertDefault(kwargs, 'filePathOrName', '.')
    print(kwargs)
    if not os.path.exists(filePathOrName):
        raise FileNotFoundError("File or folder not found.")
    fileFormat = kwargs.get('fileFormat', '').lower()
    if not fileFormat:
        # Try to determine the file format
        if os.path.isdir(filePathOrName):
            # It's a path - it could contain yarp .log or frames
            listDir = os.listdir(filePathOrName)
            fileTypes = [subName.split(".")[-1] for subName in listDir]
            mostCommonFileType = max(set(fileTypes), key=fileTypes.count)
            if mostCommonFileType == 'log':
                kwargs['fileFormat'] = 'iityarp'
            elif mostCommonFileType == 'png':
                kwargs['fileFormat'] = 'frames'
            else:
                # recurse into this folder
                resultsList = []
                for subName in listDir:
                    kwargs['filePathOrName'] = os.path.join(filePathOrName, subName)
                    try:
                        result = importAe(**kwargs)
                    except ValueError:
                        continue
                    if isinstance(result, list):
                        resultsList = resultsList + result
                    else:
                        resultsList.append(result)
                if len(resultsList) > 1 and \
                    kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)):
                        # Optional: start the timestamps at zero for the first event
                        # This is done collectively for all the concurrent imports
                        rezeroTimestampsForImportedDicts(resultsList)
                elif len(resultsList) == 1:
                    resultsList = resultsList[0]
 
                return resultsList
        else:
            # Guess the file format based on file extension
            ext = os.path.splitext(filePathOrName)[1]
            if ext == '.dat' or ext == '.raw':
                kwargs['fileFormat'] = 'dat'
            elif ext == '.bag':
                kwargs['fileFormat'] = 'rosbag'
            elif ext == '.bin':
                kwargs['fileFormat'] = 'secdvs'
            elif ext == '.npy':
                kwargs['fileFormat'] = 'iitnpy'
            elif ext == '.aer2':
                kwargs['fileFormat'] = 'aer2'
            elif ext == '.hdf5':
                kwargs['fileFormat'] = 'hdf5'
            elif ext == '.log':
                kwargs['fileFormat'] = 'iit'
            # etc ...
            else:
                raise ValueError("The file format cannot be determined.")
    # Let the fileformat parameter dictate the file or folder format
    fileFormat = kwargs.get('fileFormat').lower()
    if fileFormat in ['iityarp', 'yarp', 'iit', 'log', 'yarplog']:
        if not os.path.isdir(kwargs['filePathOrName']):
            kwargs['filePathOrName'] = os.path.dirname(kwargs['filePathOrName'])
        importedData = importIitYarp(**kwargs)
    elif fileFormat in ['rpgdvsros', 'rosbag', 'rpg', 'ros', 'bag', 'rpgdvs']:
        importedData = importRpgDvsRos(**kwargs)
    elif fileFormat in ['iitnpy', 'npy', 'numpy']:
        importedData = importIitNumpy(**kwargs)
    elif fileFormat in ['dat', 'raw']:
        importedData = importProph(**kwargs)
    elif fileFormat in ['secdvs', 'bin', 'samsung', 'sec', 'gen3']:
        importedData = importSecDvs(**kwargs)
    elif fileFormat in ['aer2']:
        importedData = importAer2(**kwargs)
    elif fileFormat in ['frame', 'frames', 'png', 'pngfolder', 'imagefolder']:
        importedData = importFrames(**kwargs)
    elif fileFormat in ['hdf5', 'bimveehdf5']:
        importedData = importHdf5(**kwargs)
    else:
        raise Exception("fileFormat: " + str(fileFormat) + " not supported.")
    #celex
    return importedData