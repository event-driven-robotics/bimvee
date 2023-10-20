# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Massimiliano Iacono
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
importInivationNumpy imports timestamped address-event data, given a path to a .npy file.
It serves the needs of a specific data format developed at Inivation using the dv library

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
    Similarly bounding boxes are saved in the channel in a separate dictionary containing the following fields:
        - ts
        - minY
        - minX
        - maxY
        - maxX
"""

import numpy as np
import os
import dv

def importInivationNumpy(filePathOrName, **kwargs):
    outDict = {
        'info': kwargs,
        'data': {}
    }
    outDict['info']['filePathOrName'] = filePathOrName

    # Importing events
    events = np.load(filePathOrName, allow_pickle=True)
    if len(events.shape) > 1:
        events = events[0]
    outDict['data']['events'] = {}
    outDict['data']['events']['dvs'] = {}

    outDict['data']['events']['dvs']['ts'] = np.array([(e.timestamp - events[0].timestamp) * 1e-6 for e in events])
    outDict['data']['events']['dvs']['x'] = np.array([e.x for e in events])
    outDict['data']['events']['dvs']['y'] = np.array([e.y for e in events])
    outDict['data']['events']['dvs']['pol'] = np.array([e.polarity for e in events])
    outDict['data']['events']['dvs']['dimX'] = max(outDict['data']['events']['dvs']['x'])  + 1
    outDict['data']['events']['dvs']['dimY'] = max(outDict['data']['events']['dvs']['y'])  + 1


    return outDict