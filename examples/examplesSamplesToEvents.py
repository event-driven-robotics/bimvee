# -*- coding: utf-8 -*-
"""
Copyright (C) 2021 Event-driven Perception for Robotics
Authors: Sim Bamford
         Simon Muller-Cleve
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation
and Export of Events etc)
This script contains a set of examples of how to use the functions of the
bimvee library.
In each case, change the file paths as required to point to your own example data.
"""

#%% Preliminaries - set your paths as necessary, or change working directory

import os, sys # A system-specific prefix, for working between linux and windows
prefix = 'C:/' if os.name == 'nt' else '/home/sbamford/'
sys.path.insert(0, os.path.join(prefix, 'repos/bimvee'))

#%% load some data

filePathOrName = os.path.join(prefix, 'data/2020_12_10_Ali_SkinExample/simeon_touch_static/samples')

from bimvee.importAe import importAe

container = importAe(filePathOrName=filePathOrName)

#%% Data format

''' 
At this point in the script we want to isolate a dict containing a single
sample datatype. Concretely, we want a dict containing at least:
'ts' - a numpy array of timestamps of type np.float64, assumed to be seconds,
monotonically increasing.
'value' (or any other field name) - a numpy array of samples
with one row (the zeroth dimension) for each timestamp, and additional
dimensions sufficient to contain the address space.
For example, if the input is from a camera,
there may be two additional dimensions for greyscale or 3 in case of RGB data.
'''

inDict = container ['data']['general']['skinSamples']

#%% convert dense samples to polarised address events

from bimvee.samplesToEvents import samplesToEvents

outDict = samplesToEvents(inDict,
                          valueKey='pressure',
                          refractoryPeriod = 0.01,
                          threshold = 40.)

#%% Visualise the result

from bimvee.plotSpikeogram import plotSpikeogram

plotSpikeogram(outDict)

#%% Try with Simon's recent data

#for m in range(5):
# Choosing material 4
m = 4

import pickle
import numpy as np

file_name = os.path.join(prefix, 
                         'data', 
                         '2021_03_09_Simon_Tactile',
                         'data_material_' + str(m+1)) #material_0 only for testing pipeline (no real data), material_1 - 5 different materials
infile = open(file_name,'rb')
dataDict = pickle.load(infile)

experimentDict = dataDict[19]
taxelData = experimentDict['taxel_data']
numSamples = taxelData.shape[0]

# Just choose one taxel
#taxelData = taxelData[:, 6]

period = 0.01
ts = np.arange(0, numSamples * period, period)

#%%
from bimvee.samplesToEvents import samplesToEvents

inDict = {'ts': ts,
          'value': taxelData}
outDict = samplesToEvents(inDict,
                          refractoryPeriod = 0.01,
                          threshold = 5.)

if 'addr' not in outDict:
    outDict['addr'] = np.zeros_like(outDict['ts'], dtype=np.int32)

#%%
from bimvee.plotSpikeogram import plotSpikeogram

axes = plotSpikeogram(outDict)

import matplotlib.pyplot as plt

axes.plot(ts, taxelData)
                
                          