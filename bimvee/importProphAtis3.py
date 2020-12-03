# -*- coding: utf-8 -*-

"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Ander Arriandiaga Laresgoiti

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
importProphAtis3 opens a file in either the .raw or .dat formats of Prophesee. 

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
from tqdm import tqdm
from struct import unpack

# Local imports
from .timestamps import zeroTimestampsForADataType

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
	event_type = unpack('b', dat.read(1))[0]
	event_size = int(unpack('b', dat.read(1))[0])

	# Compute number of events in the file
	start = dat.tell()
	dat.seek(0, 2)
	stop = dat.tell()
	dat.seek(start)
	Nevents = int((stop-start)/event_size)
	print("> The file contains %d events." % Nevents)

	ts = np.zeros((Nevents), dtype=np.float64)
	x = np.zeros((Nevents), dtype=np.uint16)
	y = np.zeros((Nevents), dtype=np.uint16)
	pol = np.zeros((Nevents), dtype=np.bool)

	for idx in tqdm(np.arange(Nevents)):
		event = dat.read(event_size)
		tsToConcatenate = int.from_bytes(event[0:4], 'little')
		raw_event = int.from_bytes(event[4:8], 'little')
		xToConcatenate = (raw_event & 0x00003FFF)
		yToConcatenate = ((raw_event & 0x0FFFC000) >> 14)
		polToConcatenate = ((raw_event & 0xF0000000) >> 28)
		ts[idx] = tsToConcatenate
		x[idx] = xToConcatenate
		y[idx] = yToConcatenate
		pol[idx] = polToConcatenate

	dvsDict = {'ts': ts / 1000000,
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

	event_size = 4  # 32-bit (4 Bytes)

	# Compute number of events in the file
	start = raw.tell()
	raw.seek(0, 2)
	stop = raw.tell()
	raw.seek(start)
	Nevents = int((stop-start)/event_size)
	print("> The file contains %d events." % Nevents)

	ts = np.zeros((Nevents), dtype=np.float64)
	x_coord = np.zeros((Nevents), dtype=np.uint16)
	y_coord = np.zeros((Nevents), dtype=np.uint16)
	pol = np.zeros((Nevents), dtype=np.bool)

	no_index_events = 0  # number count of EVT_TIME_HIGH and unneeded events.
	no_EVT_TIME_HIGH_yet = True  # was a EVT_TIME_HIGH event received yet?
	cpt = 0  # index where to store the event
	for ev in tqdm(np.arange(Nevents)):
		# read event_size bytes
		data = unpack('<I', raw.read(event_size))[0]
		# decode the type of the event
		ev_type = (data & 0xF0000000) >> 28  # event type is bits 31..28
		store_event = False  # does the event have to be stored in the vector?
		if ev_type == EVT_TIME_HIGH:
			no_index_events += 1
			no_EVT_TIME_HIGH_yet = False
			ts_MSB = (data & 0x0FFFFFFF) << 6  # MSB of the timestamp, shifted by 6 bits to make room for the LSB of the timestamp

		elif ev_type == CD_OFF:
			store_event = True
			y = data & 0x000007FF           # y is bits 10..0
			x = (data & 0x003FF800) >> 11    # x is bits 21..11
			ts_LSB = (data & 0x0FC00000) >> 22  # ts is bits 27..22
			p = 0                           # pol is 0 (LOW type)
		elif ev_type == CD_ON:
			store_event = True
			y = data & 0x000007FF           # y is bits 10..0
			x = (data & 0x003FF800) >> 11    # x is bits 21..11
			ts_LSB = (data & 0x0FC00000) >> 22  # ts is bits 27..22
			p = 1                           # pol is 0 (LOW type)
		else:  # EXT_TRIGGER, OTHERS, CONTINUED events are not needed.
			no_index_events += 1
			print('Not interested in this event type (', ev_type, ')')

		if no_EVT_TIME_HIGH_yet:
			no_index_events += 1
			store_event = False

		if store_event:
			ts[cpt] = ts_MSB + ts_LSB
			pol[cpt] = p
			x_coord[cpt] = x
			y_coord[cpt] = y
			cpt += 1

	if cpt != (Nevents - no_index_events):
		print('> Something might have gone wrong.')

	dvsDict = {'ts': ts[:cpt] / 1000000,
            'x': x_coord[:cpt],
            'y': y_coord[:cpt],
            'pol': pol[:cpt],
			   }

	return dvsDict


def importProphAtis3(**kwargs):
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
	print('Done.')

	return outDict


