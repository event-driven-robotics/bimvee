# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Author: Sim Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
plotSpikeogram received "inDict", which is a container for (2D) polarity events, 
containing fields with numpy arrays:    
    ts
    x
    y
    pol
A spikeogram has a row for each address represented, where the x dim 
represents time and vertical lines represent events. 
Color represents polarity (blue vs red for 1 vs 0)
By default, all addresses will be plotted as a row, 
but this will usually be impractical. So the kwargs
    xMin
    xMax
    yMin
    yMax
are used to constrain the space represented
and minTime and maxTime are used to constrain time. 
The axes are aligned so that the coordinate points to the middle of the first
row represented by that mark. 

TODO: There are separate functions for spikeograms from skin and dvs - this
should be generalised to all address-event datatypes (including ear, dvsLbl etc)

TODO: iterative adding of lines for each event is very inefficient for large
datasets - consider using plt.scatter.
"""

#%% Plot tsPix (single pixel timestamps)

import matplotlib.pyplot as plt
import matplotlib.lines as lines
import numpy as np
from tqdm import tqdm

# TODO: This code is very inefficient for large numbers of events
# better to iterate once for each pixel
def plotSpikeogram(inDict, **kwargs):
    # Boilerplate for descending container hierarchy
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plotSpikeogram(inDictInst, **kwargs)
        return
    if 'info' in inDict: # Top level container
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotSpikeogram was called for file ' + fileName)
        if not inDict['data']:
            print('The import contains no data.')
            return
        for channelName in inDict['data']:
            channelData = inDict['data'][channelName]
            if 'dvs' in channelData and len(channelData['dvs']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotSpikeogram(channelData['dvs'], **kwargs)
            else:
                print('Channel ' + channelName + ' skipped because it contains no polarity data')
        return
    # Break out data arrays for cleaner code
    print('plotSpikeogram working: ' + kwargs['title'])
    x = inDict['x']
    y = inDict['y']
    ts = inDict['ts']
    pol = inDict['pol']
    minX = kwargs.get('minX', np.min(x))
    maxX = kwargs.get('maxX', np.max(x))
    minY = kwargs.get('minY', np.min(y))
    maxY = kwargs.get('maxY', np.max(y))
    minTime = kwargs.get('minTime', kwargs.get('startTime', kwargs.get('beginTime', np.min(ts))))
    maxTime = kwargs.get('maxTime', kwargs.get('stopTime', kwargs.get('endTime', np.max(ts))))
    numX = maxX - minX + 1
    numY = maxY - minY + 1
    timeRange = maxTime - minTime
    selected = np.where((x >= minX) & (x <= maxX) &
                    (y >= minY) & (y <= maxY) &
                    (ts >= minTime) & (ts <= maxTime))[0]
    axes = kwargs.get('axes')
    if axes is None:
        fig, axes = plt.subplots()
        kwargs['axes'] = axes

    for idx in tqdm(selected):
        xPos = ts[idx]
        yPos = x[idx] + y[idx]*numX
        if pol[idx]:
            line = lines.Line2D([xPos, xPos], [yPos - 0.5, yPos + 0.5], 
                                color='blue', lw=2, axes=axes)
        else:
            line = lines.Line2D([xPos, xPos], [yPos - 0.5, yPos + 0.5], 
                                color='red', lw=2, axes=axes)
        axes.add_line(line)
    axes.set_xlim(minTime-timeRange*0.01, maxTime+timeRange*0.01)
    axes.set_ylim(minX + minY*numX - 1, maxX + maxY*numX + 1)
    formatString = '0'+ str(int(np.log10(999))+1) + 'd'
    numRows = numX * numY
    theRange = range(minX+minY*numX, maxX+maxY*numX, int(numRows/10)) # Let's have max 10 labels
    labels = ['x' + format(np.mod(x,numX), formatString) + ',' +
              'y' + format(int(x/numX), formatString)
              for x in theRange]
    plt.yticks(theRange, labels)
    
    callback = kwargs.get('callback')
    if callback is not None:
        callback(**kwargs)

# TODO: Generalise with above for any address-event data
def plotSpikeogramSkin(inDict, **kwargs):
    # Boilerplate for descending container hierarchy
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plotSpikeogramSkin(inDictInst, **kwargs)
        return
    if 'info' in inDict: # Top level container
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotSpikeogram was called for file ' + fileName)
        if not inDict['data']:
            print('The import contains no data.')
            return
        for channelName in inDict['data']:
            channelData = inDict['data'][channelName]
            if 'skinEvents' in channelData and len(channelData['skinEvents']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotSpikeogramSkin(channelData['skinEvents'], **kwargs)
            else:
                print('Channel ' + channelName + ' skipped because it contains no skinEvents data')
        return
    # Break out data arrays for cleaner code
    print('plotSpikeogram working: ' + kwargs['title'])
    addr = inDict['taxel'] + inDict['bodyPart'] * 2**10 # TODO: find general solution to express address of different datatypes
    ts = inDict['ts']
    pol = inDict['pol']
    # TODO: CropSpace on kwargs min/maxAddr
    minAddr = kwargs.get('minAddr', np.min(addr))
    maxAddr = kwargs.get('maxAddr', np.max(addr))
    minTime = kwargs.get('minTime', kwargs.get('startTime', kwargs.get('beginTime', np.min(ts))))
    maxTime = kwargs.get('maxTime', kwargs.get('stopTime', kwargs.get('endTime', np.max(ts))))
    timeRange = maxTime - minTime
    selected = np.where((ts >= minTime) & (ts <= maxTime))[0]
    axes = kwargs.get('axes')
    if axes is None:
        fig, axes = plt.subplots()
        kwargs['axes'] = axes

    for idx in tqdm(selected):
        xPos = ts[idx]
        yPos = addr[idx]
        if pol[idx]:
            line = lines.Line2D([xPos, xPos], [yPos - 0.5, yPos + 0.5], 
                                color='blue', lw=2, axes=axes)
        else:
            line = lines.Line2D([xPos, xPos], [yPos - 0.5, yPos + 0.5], 
                                color='red', lw=2, axes=axes)
        axes.add_line(line)
    axes.set_xlim(minTime-timeRange*0.01, maxTime+timeRange*0.01)
    axes.set_ylim(minAddr - 1, maxAddr + 1)
    formatString = '0'+ str(int(np.log10(999))+1) + 'd'
    numRows = maxAddr - minAddr + 1
    theRange = range(minAddr, maxAddr, int(np.ceil(numRows/10))) # Let's have max 10 labels
    labels = ['addr' + format(addr, formatString) for addr in theRange]
    plt.yticks(theRange, labels)
    
    callback = kwargs.get('callback')
    if callback is not None:
        callback(**kwargs)

# TODO: Generalise with above for any address-event data
def plotSpikeogramEar(inDict, **kwargs):
    # Boilerplate for descending container hierarchy
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plotSpikeogramEar(inDictInst, **kwargs)
        return
    if 'info' in inDict: # Top level container
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotSpikeogram was called for file ' + fileName)
        if not inDict['data']:
            print('The import contains no data.')
            return
        for channelName in inDict['data']:
            channelData = inDict['data'][channelName]
            if 'ear' in channelData and len(channelData['ear']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotSpikeogramEar(channelData['ear'], **kwargs)
            else:
                print('Channel ' + channelName + ' skipped because it contains no ear data')
        return
    # Break out data arrays for cleaner code
    print('plotSpikeogram working: ' + kwargs['title'])
    addr = inDict['freq'] + inDict['ch'] * 2**5 # TODO: find general solution to express address of different datatypes
    ts = inDict['ts']
    pol = inDict['pol']
    # TODO: CropSpace on kwargs min/maxAddr
    minAddr = kwargs.get('minAddr', np.min(addr))
    maxAddr = kwargs.get('maxAddr', np.max(addr))
    minTime = kwargs.get('minTime', kwargs.get('startTime', kwargs.get('beginTime', np.min(ts))))
    maxTime = kwargs.get('maxTime', kwargs.get('stopTime', kwargs.get('endTime', np.max(ts))))
    timeRange = maxTime - minTime
    selected = np.where((ts >= minTime) & (ts <= maxTime))[0]
    axes = kwargs.get('axes')
    if axes is None:
        fig, axes = plt.subplots()
        kwargs['axes'] = axes

    for idx in tqdm(selected):
        xPos = ts[idx]
        yPos = addr[idx]
        if pol[idx]:
            line = lines.Line2D([xPos, xPos], [yPos - 0.5, yPos + 0.5], 
                                color='blue', lw=2, axes=axes)
        else:
            line = lines.Line2D([xPos, xPos], [yPos - 0.5, yPos + 0.5], 
                                color='red', lw=2, axes=axes)
        axes.add_line(line)
    axes.set_xlim(minTime-timeRange*0.01, maxTime+timeRange*0.01)
    axes.set_ylim(minAddr - 1, maxAddr + 1)
    formatString = '0'+ str(int(np.log10(999))+1) + 'd'
    numRows = maxAddr - minAddr + 1
    theRange = range(minAddr, maxAddr, int(np.ceil(numRows/10))) # Let's have max 10 labels
    labels = ['addr' + format(addr, formatString) for addr in theRange]
    plt.yticks(theRange, labels)
    
    callback = kwargs.get('callback')
    if callback is not None:
        callback(**kwargs)