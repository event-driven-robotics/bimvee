# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
         Aiko Dinale
         Massimiliano Iacono
Code contributions from:
        Marco Monforte
        Daniel Gutierrez Galan
        Ali Dabbous
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
`importIitYarp` opens a `data.log` file, initially assumed to contain 24-bit encoded data from ATIS.
Returns a list of dicts, where each dict contains:
{'info': {},
 'data': {
         channel0: {} (usually the 'left' channel)
         channel1: {} (usually the 'right' channel)
         ...
         }}

Each channel is a dict containing:
    For dvs:
        - "ts": numpy array of float - seconds
        - "x": numpy array of uint16
        - "y": numpy array of uint16
        - "pol": numpy array of uint8 in [0, 1]
    For imu:
        - "ts": numpy array of float - seconds

    Can also handle LAE (labelled events), FLOW (dvs optical flow events),
    IMU (imu samples), SKE (skin events), EAR (auditory events), SKS (skin samples)

This function combines the import of several distinctly different cases:
    1) Output directly from zynqGrabber, with all events together.
        May combine samples and events in the same bottles and never contains
        explicit IMU / LAE / FLOW events. Samples such as IMU are contained
        within AE type bottles.
    2) Split and processed events from vPreProcess or other downstream modules.
        The latter usually has distinct bottles for each datatype, and channels
        have probably, but not definitely, been split into left and right.
    3) Output from binary event dumper.
    4) Data from Vicon.
    5) Skin samples which have not been properly formatted as bottles.

TODO: import frames - yarp has 2 ways of encoding - as bottles or,
recommended, by saving e.g. pngs to a folder - support the latter

There are 2 dvs codecs: 20-bit and 24-bit. 24-bit is assumed unless kwarg
"codec" = "20bit" is received

More information about event decoding are available in the event-driven reposi-
tory on GitHub:
    https://github.com/robotology/event-driven/blob/master/documentation/eventcodecs.md
"""

# %%

import re
import numpy as np
import os
import csv

# Local imports
from .importIitVicon import importIitVicon
from .timestamps import unwrapTimestamps, zeroTimestampsForAChannel, rezeroTimestampsForImportedDicts
from .split import selectByLabel
from bimvee.importBoundingBoxes import importBoundingBoxes
from bimvee.importSkeleton import importSkeleton

def decodeEvents(data, **kwargs):
    """
    Decode the binary patterns in `data` to the corresponding event type.

    Timestamps (ts) are 32 bit integers counted with an 80 ns clock - do this conversion later.
    There is then a 32 bit data payload.
    The following patterns mark the different data types (MSB left - LSB right):
        0000 0XXX XXXX XXXX: dvs
        0000 1XXX XXXX XXXX: aps
        0001 0XXX XXXX XXXX: skin
        0010 0XXX XXXX XXXX: imu
        0100 0XXX XXXX XXXX: ear
    DVS events are encoded as 32 bits with x, y, channel (ch)(c) and polarity (pol)(p)
    in two possible configuration as shown below:
        0000 0000 000c 00yy yyyy yyxx xxxx xxxp ("20-bit codec" - this codec is now deprecated but useful to load old datasets)
        0000 0000 0crr yyyy yyyy rrxx xxxx xxxp ("24 bit codec")
    ... where:
            - x and y = respective coords;
            - p = polarity;
            - r = reserved for future expansion, but practically used for larger sensor formats
            - c = channel 1 = right vs 0 = left.
    APS:
        not written yet
    IMU samples:
        rrrr rrrr tcrr ssss vvvv vvvv vvvv vvvv
        r = reserved = 0
        t = type: 1=sample
        c = channel (left vs right)
        s = sensor (uint4)
        v = value of the sample (int16)
    SKE (skin) events:
        0001 0000 TsRR Rbbb RRcR Rttt tttt tttp
        polarity(p) - bool
        taxel(t) - uint16
        cross_base(c) - bool - for events we can ignore this
        body_part(b) - uint8
        side(s) - uint8 (could be bool)
        type(T) - uint8 (could be bool) - 0 for events, 1 for samples
        reserved (R)
    EAR (Cochlear / audio) events:
        0000 0sss 0css snnn nnnn rrmo ffff fffp
        polarity          (p: 0 -> positive; 1 -> negative)
        frequency channel (f: from 0 to 31)
        olive model       (o: 0 -> MSO; 1 -> LSO)
        auditory model    (m: 0 -> cochlea; 1 -> superior olivary complex)
        reserved          (r: fixed to 0)
        ITD neurons id    (n: from 0 to 15)
        sensor ID         (s: fixed to b'100')
        channel           (c: 0 -> left; 1 -> right)
    """
    dvsBool = np.logical_not(data[:, 1] & 0xFF800000)
    apsBool = np.bool_(data[:, 1] & 0x00800000)
    skinBool = np.bool_(data[:, 1] & 0x01000000)
    imuBool = np.bool_(data[:, 1] & 0x02000000)
    earBool = np.bool_(data[:, 1] & 0x04000000)
    if kwargs.get('codec', '24bit') == '20bit':
        dvsBool[:] = True
        apsBool[:] = False
        skinBool[:] = False
        imuBool[:] = False
        earBool[:] = False

    outDict = {
        'dvs': None,
        'aps': None,
        'skinSamples': None,
        'skinEvents': None,
        'imuSamples': None,
        'ear': None,
        }
    if np.any(dvsBool):
        dataDvs = data[dvsBool, :]
        ts = dataDvs[:, 0]
        if np.isscalar(ts):
            ts = np.ones((1), dtype=np.uint32) * ts
        pol = ~np.bool_(dataDvs[:, 1] & 0x01)  # We want True=ON=brighter, False=OFF=darker, so we negate
        dataDvs[:, 1] >>= 1
        if kwargs.get('codec', '24bit') == '20bit':
            # If any non zero value shows up in bits 18-19 then we are sure that the new 24 bit codec is being used
            if np.bool_(dataDvs[:, 1] & 0x00C00000).any():
                raise ValueError("Data codec not consistent or data check not valid")
            x = np.uint16(dataDvs[:, 1] & 0x1FF)
            dataDvs[:, 1] >>= 9
            y = np.uint16(dataDvs[:, 1] & 0xFF)
            dataDvs[:, 1] >>= 10
        else:  # 24bit - default
            x = np.uint16(dataDvs[:, 1] & 0x7FF)
            dataDvs[:, 1] >>= 11
            y = np.uint16(dataDvs[:, 1] & 0x3FF)
            dataDvs[:, 1] >>= 10
        ch = np.uint8(dataDvs[:, 1] & 0x01)
        outDict['dvs'] = {
            'ts': ts,
            'ch': ch,
            'x': x,
            'y': y,
            'pol': pol}

    if np.any(imuBool):
        dataImu = data[imuBool, :]
        ts = np.uint32(dataImu[:, 0])
        if np.isscalar(ts):
            ts = np.ones((1), dtype=np.uint32) * ts
        value = np.int16(dataImu[:, 1] & 0xFFFF)
        dataImu[:, 1] >>= 16
        sensor = np.uint8(dataImu[:, 1] & 0x0F)
        dataImu[:, 1] >>= 6
        ch = np.uint8(dataImu[:, 1] & 0x01)
        outDict['imuSamples'] = {
                'ts': ts,
                'ch': ch,
                'sensor': sensor,
                'value': value}

    if np.any(skinBool):
        dataSkin = data[skinBool, :]
        ts = np.uint32(dataSkin[:, 0])
        if np.isscalar(ts):
            ts = np.ones((1), dtype=np.uint32) * ts
        polarity = np.bool_(dataSkin[:, 1] & 0x01)
        dataSkin[:, 1] >>= 1
        taxel = np.uint16(dataSkin[:, 1] & 0x3FF)
        dataSkin[:, 1] >>= 12
#        crossBase = np.uint8(dataSkin[:, 1] & 0x01)
        dataSkin[:, 1] >>= 3
        bodyPart = np.uint8(dataSkin[:, 1] & 0x07)
        dataSkin[:, 1] >>= 6
        side = np.uint8(dataSkin[:, 1] & 0x01)
#        dataSkin[:, 1] >>= 1
#        eventType = np.uint8(dataSkin[:, 1] & 0x01)
        outDict['skinEvents'] = {
                'ts': ts,
                'side': side,
                'bodyPart': bodyPart,
                'taxel': taxel,
                'pol': polarity}

    if np.any(earBool):
        dataEar = data[earBool, :]
        ts = np.uint32(dataEar[:, 0])
        if np.isscalar(ts):
            ts = np.ones((1), dtype=np.uint32) * ts
        polarity = np.bool_(dataEar[:, 1] & 0x01)
        dataEar[:, 1] >>= 1
        frequencyChannel = np.uint8(dataEar[:, 1] & 0x7F)
        dataEar[:, 1] >>= 7
        xsoType = np.uint8(dataEar[:, 1] & 0x01)
        dataEar[:, 1] >>= 1
        auditoryModel = np.uint8(dataEar[:, 1] & 0x01)
        dataEar[:, 1] >>= 3
        itdNeuronIds = np.uint8(dataEar[:, 1] & 0x7F)
        dataEar[:, 1] >>= 10
        channel = np.uint8(dataEar[:, 1] & 0x01)
        outDict['ear'] = {
                'ts': ts,
                'ch': channel,
                'itdNeuronIds': itdNeuronIds,
                'auditoryModel': auditoryModel,
                'xsoType': xsoType,
                'freq': frequencyChannel,
                'pol': polarity}
    return outDict


def getOrInsertDefault(inDict, arg, default):
    """
    Get the value specified by `arg` from the dictionary `inDict`.

    If `inDict` doesn't contain `arg`, return `default`.
    Also insert `default` into `inDict`.
    """
    value = inDict.get(arg, default)
    if value == default:
        inDict[arg] = default
    return value


def samplesToImu(inDict, **kwargs):
    """
    Group IMU samples together into multi-dimensional samples.

    `inDict` is a dictionary containing `ts`, `sensor`, and `value`.
    Assume these are IMU samples, and compile them (up to) 10 at a time.

    `outDict` is a dictionary containing `ts`, `acc`, `angV`, `temp` and `mag`.

    The `sensor` key is defined as:
        0: - (negative) Accel Y     1: Accel X     2: Accel Z
        3: - (negative) Gyro Y      4: Gyro X      5: Gyro Z
        6: Temperature
        7: - (negative) Mag Y       8: Mag X       9: Mag Z

    For the IMU on STEFI, (which is: ICM-20948):
        - gyro full scale is +/-2000 degrees per second
        - accelerometer full scale is +/-2 g
        - temp - 333.87 but does it zero at 0K or at room temperature? (+21deg)
        - mag full scale is +/- 4900 uT
    """
    accConversionFactor = kwargs.get('accConversionFactor', (32768 / 2) / 9.80665)
    angVConversionFactor = kwargs.get('angVConversionFactor', (32768 / 250) * (180 / np.pi))
    tempConversionFactor = kwargs.get('tempConversionFactor', 333.87)
    tempConversionOffset = kwargs.get('tempConversionOffset', -273.15 - 21)
    magConversionFactor = kwargs.get('magConversionFactor', (32768 / 4900) * 1000000)
    # Assume that sensor always ramps up, and split when it wraps around
    # Otherwise, iterate through this sample by sample - slowest and most robust method
    # Possible to do this much faster, but only my assuming no gaps in data
    tsAll = inDict['ts']
    sensorAll = inDict['sensor'].astype(np.int16)
    valueAll = inDict['value']
    wrapIds = np.where((sensorAll[1:]-sensorAll[:-1]) < 1)[0]
    numImu = len(wrapIds) + 1
    tsOut = np.zeros((numImu), dtype=np.float64)
    acc = np.zeros((numImu, 3), dtype=np.int16)
    angV = np.zeros((numImu, 3), dtype=np.int16)
    temp = np.zeros((numImu, 1), dtype=np.int16)
    mag = np.zeros((numImu, 3), dtype=np.int16)
    imuPtr = -1
    sensorPrev = 100
    for ts, sensor, value in zip(tsAll, sensorAll, valueAll):
        if sensor <= sensorPrev:
            imuPtr += 1
            tsOut[imuPtr] = ts  # Just take the first ts for a group of samples
        sensorPrev = sensor
        if sensor == 0:
            acc[imuPtr, 1] = -value
        elif sensor == 1:
            acc[imuPtr, 0] = value
        elif sensor == 2:
            acc[imuPtr, 2] = value
        elif sensor == 3:
            angV[imuPtr, 1] = -value
        elif sensor == 4:
            angV[imuPtr, 0] = value
        elif sensor == 5:
            angV[imuPtr, 2] = value
        elif sensor == 6:
            temp[imuPtr, 0] = value
        elif sensor == 7:
            mag[imuPtr, 1] = -value
        elif sensor == 8:
            mag[imuPtr, 0] = value
        elif sensor == 9:
            mag[imuPtr, 2] = value
    acc = acc.astype(np.float64) / accConversionFactor
    angV = angV.astype(np.float64) / angVConversionFactor
    temp = temp.astype(np.float64) / tempConversionFactor - tempConversionOffset
    mag = mag.astype(np.float64) / magConversionFactor
    outDict = {
            'ts': tsOut,
            'acc': acc,
            'angV': angV,
            'temp': temp,
            'mag': mag,
            }
    return outDict


def groupSkinSamples(inDict):
    """
    Group skin samples together into multi-dimensional samples.

    Similarly to imu data, skin samples are delivered in batches with each of
    the 192 taxels sampled once in a repeating pattern, each with its own time-
    stamp. Group these together into multi-dimensional samples each with a sin-
    gle timestamp. We use the observation that groups start with taxel 213 to
    simplify this.

    TODO: Use a more robust method of identifying groups.

    Note: We reject any data that doesn't form a complete group!

    The output dictionary includes a key `taxelOrder` so that the ordered sam-
    ples can later be mapped to their respective addresses.
    """
    wrapIds = np.where((inDict['taxel'] == 213))[0]
    pressure = inDict['value'][wrapIds[0]:wrapIds[-1]].reshape(-1, 192)
    taxelOrder = inDict['taxel'][wrapIds[0]:wrapIds[1]]
    ts = inDict['ts'][wrapIds[0]:wrapIds[-1]:192]
    return {'ts': ts,
            'pressure': pressure,
            'taxelOrder': taxelOrder}


def appendBatch(mainDict, batch):
    """
    Append `batch` of events to the dictionary `mainDict`.

    For a batch of events for a given `dataType`, and the container of all
    events for that `dataType` so far, add the batch to the container.

    The first batch for each data type gets used as the model to which all
    further batches get added.

    A container is a dict of numpy arrays. As these arrays need to be expanded
    to fit the new data, they are doubled in size. This limits to O(log(n)) the
    number of times we need to resize the arrays and it limits to nx4 the amount
    of memory used. An alternative would be to build lists and then convert to
    numpy at the end.
    """
    if batch is None:
        return mainDict
    numEventsBatch = len(batch['ts'])
    if mainDict is None:
        batch['numEvents'] = numEventsBatch
        return batch
    # Check if the main array has enough free space for this batch
    sizeOfMain = len(mainDict['ts'])
    numEventsMain = mainDict['numEvents']
    while numEventsBatch + numEventsMain > sizeOfMain:
        for fieldName in mainDict.keys():
            if fieldName != 'numEvents':
                # Double the size of the array
                mainDict[fieldName] = np.append(mainDict[fieldName], np.empty_like(mainDict[fieldName]), axis=0)
        sizeOfMain *= 2
    # Add the batch into the main array
    for fieldName in batch.keys():
        if mainDict[fieldName].ndim == 2:
            mainDict[fieldName][numEventsMain:numEventsMain + numEventsBatch, :] = \
            batch[fieldName]
        else:
            mainDict[fieldName][numEventsMain:numEventsMain + numEventsBatch] = \
            batch[fieldName]
    mainDict['numEvents'] = numEventsMain + numEventsBatch
    return mainDict


def cropArraysToNumEvents(inDict):
    """Crop each array of `inDict` up to the number of events in `inDict`."""
    if inDict is None:
        return None
    numEvents = inDict.pop('numEvents')
    for fieldName in inDict.keys():
        inDict[fieldName] = inDict[fieldName][:numEvents]
    return inDict


def globalPostProcessing(inDict, **kwargs):
    """
    Carry out global post-processing steps on the imported dictionary `inDict`.

    Main steps:
        1) Zero timestamps.
        2) Split by channel - The iit yarp format assumes that the channel bit
           corresponds to `left` and `right` sensors, so it's handled explici-
           tly here.
        3) Construct output dictionary.
    """
    # zero time by default (keeping the offset)
    if getOrInsertDefault(kwargs, 'zeroTimestamps', True):
        zeroTimestampsForAChannel(inDict)
    # Split into left and right channels
    # Allowing previously used 'split_stereo' for backwards compatibility
    splitByChannel = kwargs.get('splitChannel', kwargs.get('split_stereo', True))
    if splitByChannel:
        dataTypes = list(inDict.keys())
        channels = {}
        for chIdx, chName in enumerate(['left', 'right']):
            chDict = {}
            for dataType in dataTypes:
                dataForChannel = None
                if 'ch' in inDict[dataType]:
                    dataForChannel = selectByLabel(inDict[dataType], 'ch', chIdx)
                elif 'side' in inDict[dataType]:
                    dataForChannel = selectByLabel(inDict[dataType], 'side', chIdx)
                if dataForChannel is not None:
                    chDict[dataType] = dataForChannel
            if any(chDict):
                channels[chName] = chDict
        for dataType in dataTypes:
            if 'ch' in inDict[dataType] or 'side' in inDict[dataType]:
                del inDict[dataType]
        if any(inDict):
            channels['general'] = inDict
    else:
        channels = {'all': inDict}
    # Construct the output dict
    outDict = {
        'info': kwargs,
        'data': channels
        }
    outDict['info']['fileFormat'] = 'iityarp'
    return outDict


def importPostProcessing(inDict, **kwargs):
    """
    Carry out post-processing steps on the imported dictionary `inDict`.

    These are common steps for data imported both from `data.log` and from bin-
    ary file.

    Main steps:
        1) Eliminate dict keys which have no data
        2) Unwrap timestamps and convert to seconds
        3) IMU data special handling - convert samples to IMU (because up to 10
           consecutive samples define single imu read-out)
        4) Skin data special handling - ...
        5) Call to global post processing for splitting by channel, timestamp
           zeroing and formation as a top-level container.
    """
    # List out the datatypes here because the for loop changes the dict
    dataTypes = list(inDict.keys())
    for dataType in dataTypes:
        # Eliminate dataTypes which didn't have any events
        if inDict[dataType] is None or not dataType == 'dvs': # TODO find a better way to handle not dvs data
            del inDict[dataType]
        else:
            # Iron out any time-wraps which occurred and convert to seconds
            isGen1 = (inDict[dataType]['x'] < 346).all() and (inDict[dataType]['y'] < 260).all()
            clock_time = 0.00000008 if isGen1 else 0.000001
            inDict[dataType]['ts'] = unwrapTimestamps(inDict[dataType]['ts'],
                                                      **kwargs) * clock_time
            # Special handling for imu data
            if dataType == 'imuSamples':
                if kwargs.get('convertSamplesToImu', True):
                    inDict['imu'] = samplesToImu(inDict.pop('imuSamples'), **kwargs)
                else:
                    inDict['sample'] = inDict.pop('imuSamples')
                    # TODO: The name "sample" is too vague
            # Special handling for skin data
            if dataType == 'skinSamples':
                inDict['skinSamples'] = groupSkinSamples(inDict['skinSamples'])
    return globalPostProcessing(inDict, **kwargs)


def importIitYarpBinaryDataLog(**kwargs):
    """Import data in IIT Yarp format from a binary file."""
    importFromByte = kwargs.get('importFromByte', 0)
    importMaxBytes = kwargs.get('importMaxBytes')
    if importMaxBytes is not None:
        importMaxBytes -= np.mod(importMaxBytes, 8)  # Events are 8 bytes long
    with open(kwargs['filePathOrName'], 'rb') as file:
        file.seek(importFromByte)
        if importMaxBytes is not None:
            events = file.read(importMaxBytes)
            # TODO: check if importMaxBytes is "feasible" (not beyond the file size)?
        else:
            events = file.read()
        events = np.frombuffer(events, np.uint32)
        events = events.reshape((-1, 2))
        outDict = decodeEvents(events)
        if file.read(1) is None:
            kwargs['importedToByte'] = 'EOF'
        else:
            kwargs['importedToByte'] = file.tell() - 2
            # - 2 compensates for the small read-ahead just performed to test EOF
    return importPostProcessing(outDict, **kwargs)


def importIitRawSkinSamples(**kwargs):
    """
    Import raw skin samples expressed in IIT Yarp format.

    Some files from the 'Old MTB' (As of 2020_12 mounted on iCub) where skin is
    dumped it creates a quite different bottle format:
        bottleNumber(int - actually alway -1) ts(float already in seconds) sample1(float) sample2(float) ...
    """
    importFromByte = kwargs.get('importFromByte', 0)
    importToByte = kwargs.get('importMaxBytes')
    if importToByte is not None:
        importToByte += importFromByte
    with open(kwargs['filePathOrName'], 'r') as file:
        file.seek(importFromByte)
        importedToByte = 'EOF'  # default if the following loop exits normally
        line = file.readline()
        currentPointer = 0
        skinSamples = None
        while line:
            if importToByte is not None:
                if file.tell() > importToByte:
                    importedToByte = currentPointer - 1
                    break
                else:
                    currentPointer = file.tell()
            try:
                numbers = line.split(' ')
                numSamples = len(numbers) - 2
                # TODO: Later, we might generalise the number of channels.
                assert numSamples == 192
                ts = float(numbers[1])
                samples = np.array(numbers[2:], dtype=np.float64)[np.newaxis, :]
                newSample = {'ts': np.ones((1), dtype=np.float64) * ts,
                             'pressure': samples}
                skinSamples = appendBatch(skinSamples, newSample)
            except ValueError:  # sometimes finding malformed packets at the end of files - ignoring
                print('Failed to interpret a line')
                line = file.readline()
                continue
            line = file.readline()
    skinSamples['ts'] = unwrapTimestamps(skinSamples['ts'])  # will this ork on float values?
    outDict = {'skinSamples': cropArraysToNumEvents(skinSamples)}
    kwargs['importedToByte'] = importedToByte
    return globalPostProcessing(outDict, **kwargs)


def importIitYarpDataLog(**kwargs):
    """Import data in IIT Yarp format from a `data.log` file."""
    # Check if format suggests data from vicon dumper
    patternForVicon = re.compile('(\d+) (\d+\.\d+) \((.*)\)')
    with open(kwargs['filePathOrName'], 'r') as inFile:
        content = inFile.readline()  # Look at first line of file
    if patternForVicon.findall(content):
        print('Yarp vicon dumper pattern found - passing this file to importVicon function')
        return importIitVicon(**kwargs)
    patternForRawSkinSamples = re.compile('(\d+) (\d+\.\d+) (\d+\.\d+)')
    if patternForRawSkinSamples.findall(content):
        print('Pattern found seems to be a raw dump of analogue skin samples')
        return importIitRawSkinSamples(**kwargs)
    # Create dicts for each possible datatype
    dvs = None
    dvsLbl = None
    dvsFlow = None
    imuSamples = None  # Imu Sample is an intermediate datatype - later it gets converted to IMU etc
    skinEvents = None
    skinSamples = None
    ear = None
    aps = None
    pattern = re.compile('(\d+) (\d+\.\d+) ([A-Z]+) \((.*)\)')
    importFromByte = kwargs.get('importFromByte', 0)
    importToByte = kwargs.get('importMaxBytes')
    if importToByte is not None:
        importToByte += importFromByte
    with open(kwargs['filePathOrName'], 'r') as file:
        file.seek(importFromByte)
        importedToByte = 'EOF'  # default if the following loop exits normally
        line = file.readline()
        currentPointer = 0
        while line:
            if importToByte is not None:
                if file.tell() > importToByte:
                    importedToByte = currentPointer - 1
                    break
                else:
                    currentPointer = file.tell()
            found = pattern.match(line)
            # The following values would be useful for indexing the input file:
            # bottlenumber = np.uint32(found[1])
            # timestamp = np.float64(found[2])
            bottleType = found[3]
            if bottleType not in ['AE', 'IMUS', 'LAE', 'FLOW', 'EAR', 'SKS', 'SKE']:
                print('Unknown bottle type: ' + bottleType)
                line = file.readline()
                continue
            try:
                events = np.array(found[4].split(' '), dtype=np.uint32)
                '''
                The following block handles unusual bottle format for labelled
                events and flow events, which all end up being treated as
                'dvs' type events. In those cases there are extra words in
                the bottle for each event.
                '''
                if bottleType == 'LAE':
                    numEventsInBatch = int(len(events) / 3)
                    events = events.reshape(numEventsInBatch, 3)
                    outDict = decodeEvents(events[:, :2], **kwargs)
                    dvsBatch = outDict['dvs']
                    dvsBatch['lbl'] = events[:, 2]
                    dvsLbl = appendBatch(dvsLbl, dvsBatch)
                elif bottleType == 'FLOW':
                    numEventsInBatch = int(len(events) / 4)
                    events = events.reshape(numEventsInBatch, 4)
                    outDict = decodeEvents(events[:, :2], **kwargs)
                    dvsBatch = outDict['dvs']
                    dvsBatch['vx'] = events[:, 2].view(dtype=np.float32)
                    dvsBatch['vy'] = events[:, 3].view(dtype=np.float32)
                    dvsFlow = appendBatch(dvsFlow, dvsBatch)
                elif bottleType == 'SKS':
                    ''' Skin samples are encoded with 4 words:
                        [ts, address, ts, analogue_sample(least-sig. 16 bits)]
                        The address is encoded the same as for skin events
                        we ignore the second timestamp
                    '''
                    numEventsInBatch = int(len(events) / 4)
                    events = events.reshape(numEventsInBatch, 4)
                    outDict = decodeEvents(events, **kwargs)
                    skinSampleBatch = outDict.pop('skinEvents')  # pop these so they don't get treated as skin events, below
                    skinSampleBatch['value'] = np.mod(events[:, 3], 2**16)
                    skinSamples = appendBatch(skinSamples, skinSampleBatch)
                else:  # bottleType in ['AE', 'IMUS', 'SKE', 'EAR', 'APS']
                    numEventsInBatch = int(len(events) / 2)
                    events = events.reshape(numEventsInBatch, 2)
                    outDict = decodeEvents(events[:, :2], **kwargs)
                    dvsBatch = outDict['dvs']
                    dvs = appendBatch(dvs, dvsBatch)
                    imuSamples = appendBatch(imuSamples, outDict['imuSamples'])
                    ear = appendBatch(ear, outDict['ear'])
                    skinEvents = appendBatch(skinEvents, outDict['skinEvents'])
                    aps = appendBatch(aps, outDict['aps'])
            except ValueError:  # sometimes finding malformed packets at the end of files - ignoring
                line = file.readline()
                continue
            line = file.readline()
    # If importedToByte is not defined then we reached the end of the file
    # Crop arrays to number of events
    outDict = {
        'dvs': cropArraysToNumEvents(dvs),
        'dvsLbl': cropArraysToNumEvents(dvsLbl),
        'flow': cropArraysToNumEvents(dvsFlow),  # TODO: 'flow' is a poor name, considering we also have dense flow maps
        'imuSamples': cropArraysToNumEvents(imuSamples),  # Imu Sample is an intermediate datatype - later it gets converted to IMU etc
        'skinEvents': cropArraysToNumEvents(skinEvents),
        'skinSamples': cropArraysToNumEvents(skinSamples),
        'ear': cropArraysToNumEvents(ear),
        'aps': cropArraysToNumEvents(aps),
        }
    kwargs['importedToByte'] = importedToByte
    return importPostProcessing(outDict, **kwargs)


def importIitYarpInfoLog(**kwargs):
    """
    Import `info.log` from data collected in IIT Yarp format.

    From the `info.log` we want the time that the channel connected; we assume
    this is the first decimal found.
    """
    # Read file
    filePathOrName = kwargs.get('filePathOrName')
    print('Examining info.log: ' + filePathOrName)
    with open(filePathOrName, 'r') as inFile:
        content = inFile.read()
    patternForInfoLog = re.compile('(\d+\.\d+)')
    found = patternForInfoLog.search(content)
    if found:
        tsOffset = float(found[0])
        return tsOffset


def importIitYarpRecursive(**kwargs):
    """
    Import data in IIT Yarp format in a recursive manner.

    This function works in the following way for efficiency when importing very
    large files:
    - Import events bottle by bottle
    - Create arrays to hold the first 1024 values then extend the array size
      exponentially while importing, finally cropping these arrays.

    Note: `kwargs` is augmented where necessary and becomes the "info" dict of
    the output.
    """
    path = getOrInsertDefault(kwargs, 'filePathOrName', '.')
    print('importIitYarp trying path: ' + path)
    if not os.path.exists(path):
        raise FileNotFoundError("path not found.")
    if not os.path.isdir(path):
        raise FileNotFoundError("path is not a directory.")
    files = sorted(os.listdir(path))
    importedDicts = []
    tsOffset = None
    boundingBoxes = None
    skeleton = None
    for file in files:
        filePathOrName = os.path.join(path, file)
        kwargs['filePathOrName'] = filePathOrName
        if os.path.isdir(filePathOrName):
            importedDicts = importedDicts + importIitYarpRecursive(**kwargs)
        if file == 'binaryevents.log':
            importedDicts.append(importIitYarpBinaryDataLog(**kwargs))
        if file == 'data.log':
            importedDicts.append(importIitYarpDataLog(**kwargs))
        if file == 'info.log':
            tsOffset = importIitYarpInfoLog(**kwargs)
        if file == 'ground_truth.csv':
            boundingBoxes = importBoundingBoxes(**kwargs)
        if file == 'skeleton.json':
            skeleton = importSkeleton(**kwargs)
    if len(importedDicts) == 0:
        print('    "data.log" file not found')
        return importedDicts
    if tsOffset is not None:
        importedDicts[-1]['info']['tsOffsetFromInfo'] = tsOffset
    if boundingBoxes is not None:
        addGroundTruth(boundingBoxes, importedDicts, 'boundingBoxes')
    if skeleton is not None:
        addGroundTruth(skeleton, importedDicts, 'skeleton')
    return importedDicts


def addGroundTruth(groundTruth, importedDicts, name):
    if len(importedDicts) == 1:
        keys = list(importedDicts[-1]['data'])
        if len(keys) == 1:
            importedDicts[-1]['data'][keys[0]][name] = groundTruth
        else:
            # TODO If more than one channel is present we don't know which one to assign the ground truth
            print(f'Found channels {keys}. Don\'t know which one to assign ground truth. Skipping.')


def importIitYarp(**kwargs):
    """Import data in IIT Yarp format."""
    importedDicts = importIitYarpRecursive(**kwargs)
    if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)):
        # Optional: start the timestamps at zero for the first event
        # This is done collectively for all the concurrent imports
        rezeroTimestampsForImportedDicts(importedDicts)
    if len(importedDicts) == 1:
        importedDicts = importedDicts[0]
    return importedDicts
