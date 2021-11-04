# -*- coding: utf-8 -*-

"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Ander Arriandiaga Laresgoiti
        Sim Bamford

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
importProph opens a file in either the .raw or .dat formats of Prophesee. 

https://docs.prophesee.ai/data_formats/file_formats/dat.html
https://docs.prophesee.ai/data_formats/data_encoding_formats/evt2.html

Returns a dict in this format:
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
from struct import unpack

# Local imports
from .timestamps import zeroTimestampsForADataType
from bimvee.importBoundingBoxes import importBoundingBoxes
import os

def importDatHeaders(dat):

    # Go to the beginning of the file
    dat.seek(0)

    info = {}

    # Assume the format version is 1 unless a header of the version number is
    # found
    info['fileFormat'] = 1

    # Read the first character
    is_comment = '%' in str(dat.read(1))
    while is_comment:

        # Read the rest of the line
        line = dat.readline().decode('utf-8', "ignore")
        # File format
        if line[22: 24] == 'CD':
            info['fileFormat'] = line[22: 24]

        # Version
        if line[: 9] == ' Version ':
            info['version'] = line[9:-1]
        # Pick out date and time of recording
        #  % Date YYY-MM-DDHH:MM:SS
        if line[: 6] == ' Date ':
            info['dateTime'] = line[6:-1]

        # Horizontal size of image sensor array
        if line[: 7] == ' Width ':
            info['Width'] = line[7:-1]

        # Horizontal size of image sensor array
        if line[: 8] == ' Height ':
            info['Height'] = line[8:-1]

        # Read ahead the first character of the next line to complete the
        # while loop
        is_comment = '%' in str(dat.read(1))

    # We have read ahead one byte looking for '#', and not found it.
    # Now wind back one to be in the right place to start reading
    dat.seek(-1, 1)

    return info


def importDat(dat):

    # Binary event type & size
    # This information is provided as two bytes.
    # First one indicates event type: for now, only CD type is supported (0x00).
    # The second provide the Event size: always 8
    try:
        event_type = unpack('b', dat.read(1))[0]
        assert event_type == 12
        
        event_size = int(unpack('b', dat.read(1))[0])
        assert event_size == 8
    except AssertionError:
        print('This file format not supported by this decoder.')
        raise
    
    events = dat.read()
    events = np.frombuffer(events, np.uint32().newbyteorder('<'))
    events = events.reshape(-1, 2)
    ts = events[:,0].astype(np.float64) / 1000000
    events = events[:,1]
    x = (events & 0x00003FFF).astype(np.int16)
    y = ((events & 0x0FFFC000) >> 14).astype(np.int16)
    pol = (events & 0xF0000000) >> 28
    
    try:
        assert np.max(pol) < 2 # why are there 4 bits for polarity? Is this for TD events?
    except AssertionError:
        print('This file format not supported by this decoder.')
        raise
    pol = pol.astype(np.bool)
    
    dvsDict = {'ts': ts,
               'x': x,
               'y': y,
               'pol': pol,
               }
    
    return dvsDict


def importRawHeaders(raw):

    # Go to the beginning of the file
    raw.seek(0)
    info = {}
    # Read the first character
    is_comment = '%' in str(raw.read(1))
    while is_comment:
        # Read the rest of the line
        line = raw.readline().decode('utf-8', "ignore")
        # Pick out date and time of recording
        #  % Date YYY-MM-DDHH:MM:SS
        if line[: 6] == ' Date ':
            info['dateTime'] = line[6:-1]
        # Integrator name
        if line[: 17] == ' integrator_name ':
            info['IntegratorName'] = line[17:-1]
        # Plugin name
        if line[: 13] == ' plugin_name ':
            info['PluginName'] = line[13:-1]
        # Serial Number
        if line[: 15] == ' serial_number ':
            info['SerialNumber'] = line[15:-1]
        # EVT
        if line[: 5] == ' evt ':
            info['evt'] = line[5:-1]
        # Firmware Version
        if line[: 18] == ' firmware_version ':
            info['FirmwareVersion'] = line[18:-1]
        # System ID
        if line[: 11] == ' system_ID ':
            info['SystemID'] = line[11:-1]
        # Subsystem ID
        if line[: 14] == ' subsystem_ID ':
            info['SubsystemID'] = line[14:-1]
        # Horizontal size of image sensor array
        if line[: 7] == ' Width ':
            info['Width'] = line[7:-1]
        # Horizontal size of image sensor array
        if line[: 8] == ' Height ':
            info['Height'] = line[8:-1]
        # Read ahead the first character of the next line to complete the
        # while loop
        is_comment = '%' in str(raw.read(1))
    # We have read ahead one byte looking for '#', and not found it.
    # Now wind back one to be in the right place to start reading
    raw.seek(-1, 1)
    
    return info


def importRaw(raw):
    """ Read .raw binary files which are direct dumps of the ATIS3 USB stream.
    (not kAER _td.dat files)
    Event format is of the evt 2.0, the small footprint ATIS used in ECOMODE.
    
    Beware: this function only returns CD_OFF, CD_ON and LEFT_TD_HIGH events.
    NO EXT_TRIGGER. NO OTHERS.

    Each event is of size 32 bits (or 4 bytes).
    The enclosed information depends on the event type. See doc. """

    # Types of events outputed by the ATIS3
    CD_OFF = 0
    CD_ON = 1
    EVT_TIME_HIGH = 8
    EXT_TRIGGER = 12
    OTHERS = 14
    CONTINUED = 15

    data = raw.read()
    events = np.frombuffer(data, np.uint32().newbyteorder('<'))
    ev_type = (events & 0xF0000000) >> 28
    ts_MSB = (events & 0x0FFFFFFF) << 6  # MSB of the timestamp, shifted by 6 bits to make room for the LSB of the timestamp
    # Spread the TS_MSB events through to the following events
    eventIsTsHigh = ev_type == EVT_TIME_HIGH
    tsHighIds = np.where(eventIsTsHigh)[0]
    tsHighIds = np.append(tsHighIds, len(events))
    for startIdx, endIdx in zip(tsHighIds[:-1], tsHighIds[1:]):
        ts_MSB[startIdx : endIdx] = ts_MSB[startIdx]
    if tsHighIds[0] > 0:
        # wind back the clock by one ts_MSB for the initial events
        # it's not really defined but it's a safe assumption
        ts_MSB[0 : tsHighIds[0]] = ts_MSB[tsHighIds[0]] - 64
    # Eliminate ts_MSB events
    events = events[~eventIsTsHigh]
    ts_MSB = ts_MSB[~eventIsTsHigh]
    ev_type = ev_type[~eventIsTsHigh]

    eventIsOtherType = ev_type > 1
    if np.any(eventIsOtherType):
        print('Other event types are present in this file, which are not being decoded')
    events = events[~eventIsOtherType]
    ts_MSB = ts_MSB[~eventIsOtherType]
    pol = (ev_type[~eventIsOtherType]).astype(np.bool)
    # unpack the rest of the data
    y = (events & 0x000007FF).astype(np.int16)           # y is bits 10..0
    x = ((events & 0x003FF800) >> 11).astype(np.int16)    # x is bits 21..11
    ts_LSB = (events & 0x0FC00000) >> 22  # ts is bits 27..22
    ts = (ts_MSB + ts_LSB) / 1000000

    dvsDict = {'ts': ts,
        'x': x,
        'y': y,
        'pol': pol,
        }

    return dvsDict



def importProph(**kwargs):
    filePathOrName = kwargs['filePathOrName']
    if filePathOrName[-4:] == '.dat':

        print('Attempting to import ' + filePathOrName + ' as ATIS3 dat file')
        with open(filePathOrName, 'rb') as file:
            file_info = importDatHeaders(file)
            dvsDict = importDat(file)
    elif filePathOrName[-4:] == '.raw':
        print('Attempting to import ' + filePathOrName + ' as ATIS3 USB dumb raw file')
        with open(filePathOrName, 'rb') as file:
            file_info = importRawHeaders(file)
            dvsDict = importRaw(file)

    if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)):
        zeroTimestampsForADataType(dvsDict)
    outDict = {
        'info': {'filePathOrName': filePathOrName,
                 'fileFormat': 'Atis3DAT'},
        'data': {
            'ch0': {
                'dvs': dvsDict
            }
        }
    }
    outDict['info'].update(file_info)
    print('Done.')
    gt_file = os.path.join(os.path.dirname(filePathOrName), 'ground_truth.csv')
    if os.path.exists(gt_file):
        outDict['data']['ch0']['boundingBoxes'] = importBoundingBoxes(filePathOrName=gt_file)
    return outDict

# Legacy name for importProph
def importAtis3(**kwargs):
    return importProph(**kwargs)
