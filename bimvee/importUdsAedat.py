# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Sim Bamford
following https://github.com/jpdominguez/pyNAVIS/blob/master/src/pyNAVIS/loaders.py
    by Juan Pedro Dominguez
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
importUdsAedat is a function for importing timestamped address-event data, given
a path or a file containing an aedat file in the format used by PyNavis.
The output is a file-level container in bimvee format.
In general this is a dict containing:
    - info
    - data
    info: will contain minimally:
        - filePathAndName
        - fileFormat
    data: this is a list of dicts, one for each sensor or "channel" which has 
    been imported. Bear in mind that sub-functions may optionally split or join 
    channels. Within each dict, there is a field for each type of data contained.
    Each of these fields is a dict containing a numpy array for timestamps 'ts'
    (one row per event), and a np array for each type of data, with a row for 
    each of those timestamps.
    In particular there will be at least an 'ear' field, containing at least:
        ts            (timestamp)
        freq          (frequency channel, incresing addr -> decreasing frequency)
    and possibly:
        pol           (polarity 0 -> positive; 1 -> negative)
        ch            (channel: 0 -> left; 1 -> right)
        xso           (Olive model: 0 -> MSO; 1 -> LSO)
        auditoryModel (auditory model: 0 -> cochlea; 1 -> superior olivary complex)
        itdNeuron     (Addresses of Interaural-time-difference neurons)
        
There are basic formats corresponding to Ini Aedat v1 and 2.
Files don't necessarily contain headers to disambiguate these, therefore the
user must specify in kwargs any deviations from the following assumptions:
    (loosely following https://github.com/jpdominguez/pyNAVIS/blob/master/src/pyNAVIS/main_settings.py)
    codec: 'Addr2Bytes' i.e. [address(2-bytes) timestamp(4-bytes)] default
              'Addr4Bytes' i.e. [address(4-bytes) timestamp(4-bytes)]
    stereo = True
    splitByChannel = False (i.e. left vs right into separate channel dicts)
    numChannelBits: 5 (number of bits used for frequency channel addresses per sensor)
    tsTick: 1e-6 (i.e. an integer time increment corresponds to 1 microsecond)
    polarised: True (i.e. decode polarity of spikes)
    zeroTime: True (whether to offset timestamps so that they start with zero)
    
    The address is interpretted as: 0...0cf...fp    where:
        p is the polarity bit and may or may not be present
        f are the frequency bits - there are ceil(log_2(numChannels))of these
        c is the channel bit - left vs right, and may or may not be present.
    
"""

#%%

import os
import numpy as np

# local imports
from .timestamps import zeroTimestampsForADataType
from .split import splitByLabel

def importUdsAedat(**kwargs):
    filePathOrName = kwargs.get('filePathOrName')
    if not os.path.exists(filePathOrName):
        raise FileNotFoundError("File or folder not found.")
    # Move forward assuming that it's a single file
    # TODO: implement hierarchical descent through folders
    codec = kwargs.get('codec', 'addr4Bytes')
    addrSize = 4 if codec == 'addr4Bytes' else 2
    importFromByte = kwargs.get('importFromByte', 0)
    importMaxBytes = kwargs.get('importMaxBytes')
    headers = ''
    with open(filePathOrName, 'rb') as file:
        file.seek(importFromByte)

        ## Check header ##
        lt = file.readline()
        while lt and lt[0] == ord("#"):
            headers = headers + lt
            importFromByte += len(lt)
            lt = file.readline()
        file.seek(0, 2)
        eof = file.tell()
        # TODO: headers to info

        # Establish how far ahead to read
        if importMaxBytes is None:
            importMaxBytes = eof - importFromByte
        else:
            importMaxBytes -= np.mod(importMaxBytes, addrSize + 4) # Import a multiple of the word size
            # Check if importMaxBytes is "feasible" (not beyond the file size)?
            if eof > importFromByte + importMaxBytes:
                importMaxBytes = eof - importFromByte
                
        # Now Read from the start point
        file.seek(importFromByte)
        events = file.read(importMaxBytes)
        
        # Pass out where we got to
        if file.read(1) is None:
            kwargs['importedToByte'] = 'EOF'
        else:
            kwargs['importedToByte'] = file.tell() - 2
            # - 2 compensates for the small read-ahead just performed to test EOF

        # Convert to uint16 - it's the highest common denominator for the address format possibilities. 
#           dt = np.dtype(int)
#>>> dt = dt.newbyteorder('>')
#>>> np.frombuffer(buf, dtype=dt) 
 
        events = np.frombuffer(events, np.uint8)
        # The following is one way to recover big-endian order while still using np.frombuffer
        # TODO: feels like there should be a better way
        events = events.reshape((-1, addrSize + 4)).astype(np.uint32)
        tsBytes = events[:, -4:]
        ts = tsBytes[:, -1]
        for byteIdx in range(3):
            ts = ts + tsBytes[:, byteIdx] * 2 ** (8 * (4 - byteIdx))
        #ts = events[:, -2].astype(np.uint32) * 2**16 + events[:, -1]
        earDict = {'ts': ts}
        if addrSize == 4:
            addr = (events[:, 0] * 2**24 +
                    events[:, 1] * 2**16 +
                    events[:, 2] * 2**8 +
                    events[:, 3])
        else:
            addr = (events[:, 0] * 2**8 +
                    events[:, 1])
        if kwargs.get('polarised', ('polarized', True)):
            earDict['pol'] = np.bool_(addr & 0x01)
            addr = addr >> 1
        numChannelBits = kwargs.get('numChannelBits', 5)
        earDict['freq'] = np.uint16(addr & (2**numChannelBits -1))
        addr >>= numChannelBits
        if kwargs.get('stereo', True):
            earDict['ch'] = np.uint8(addr & 0x01)
#                'itdNeuronIds': itdNeuronIds,
#                'auditoryModel': auditoryModel,
#                'xsoType': xsoType,
#        xsoType = np.uint8(addr & 0x01)
#        addr >>= 1
#        auditoryModel = np.uint8(addr & 0x01)
#        addr >>= 3
#        itdNeuronIds = np.uint8(addr & 0x7F)
#        addr >>= 10
    if kwargs.get('zeroTimestamps', True):
        zeroTimestampsForADataType(earDict) # TODO: This should return a new dict
    tsTick = kwargs.get('tsTick', 1e-6)
    earDict['ts'] = earDict['ts'].astype(np.float64) * tsTick
    if kwargs.get('stereo', True) and kwargs.get('splitByChannel', True):
        channels = splitByLabel(earDict, 'ch')
        if 0 in channels.keys():
            channels['left'] = {'ear': channels.pop(0)}
        if 1 in channels.keys():
            channels['right'] = {'ear': channels.pop(1)}
    else:
        channels = {'general': {'ear': earDict}}

    if headers == '':
        kwargs['headers'] = headers
    outDict = {
        'info': kwargs,
        'data': channels
        }
    outDict['info']['fileFormat'] = 'iityarp'
    return outDict
