# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
        Suman Ghosh
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
plotPose takes 'inDict' - a dictionary containing imported pose data 
(or a higher level container, in which attempts to descend and call itself) 
as created by importAe, and plots against time the various dimensions of the 
samples contained. Each dimension is plotted against time, 
and each channel has a separate figure

plotPose3d works differently; it takes a dict with multiple channels, rejects
time and rotation info and plots the point data in a 3d space, with one set of
points for each channel of dataType pose6q or point3, in a different colour. 

plotPoseMultiChannel uses the same inputs as plotPose3d to put multiple channels
of pose or point data on the same graph.

Also handles point3 type, which contains 3d points
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import itertools

# local imports
from .split import cropTime
from bimvee.pose import canoniseQuaternions, makeQuaternionsContinuous

def plotPose(inDicts, **kwargs):
    if isinstance(inDicts, list):
        for inDict in inDicts:
            plotPose(inDict, **kwargs)
        return
    else:
        inDict = inDicts
    if not isinstance(inDict, dict):
        return
    if 'ts' not in inDict:
        title = kwargs.pop('title', '')
        if 'info' in inDict and isinstance(inDict, dict):
            fileName = inDict['info'].get('filePathOrName')
            if fileName is not None:
                print('plotPose was called for file ' + fileName)
                title = (title + ' ' + fileName).lstrip()
        for key in inDict.keys():
            kwargs['title'] = (title + ' ' + key).lstrip()
            plotPose(inDict[key], **kwargs)
        return
    # From this point onwards, it's a data-type container
    if 'point' not in inDict:
        return
    # From this point onwards, it's a pose or point data-type container    

    ts = inDict['ts']
    minTime = kwargs.get('minTime', kwargs.get('startTime', kwargs.get('beginTime', ts.min())))
    maxTime = kwargs.get('maxTime', kwargs.get('stopTime', kwargs.get('endTime', ts.max())))
    if minTime > ts.min() or maxTime < ts.max():
        inDict = cropTime(inDict, minTime=minTime, maxTime=maxTime, zeroTime=False)
        ts = inDict['ts']

    axesR = kwargs.get('axesR')
    axesT = kwargs.get('axesT')
    label = kwargs.get('label', '')
    lineStyle = kwargs.get('lineStyle', '-')
    if 'rotation' in inDict:
        if axesR is None:
            fig, allAxes = plt.subplots(2, 1)
            fig.suptitle(kwargs.get('title', ''))
            axesT = allAxes[0]
            axesR = allAxes[1]
        if kwargs.get('canonise', False):
            inDict = canoniseQuaternions(inDict)
        if kwargs.get('makeContinuous', True):
            inDict = makeQuaternionsContinuous(inDict)
        rotation = inDict['rotation']
        axesR.plot(ts, rotation[:, 0], 'k' + lineStyle)
        axesR.plot(ts, rotation[:, 1], 'r' + lineStyle)
        axesR.plot(ts, rotation[:, 2], 'g' + lineStyle)
        axesR.plot(ts, rotation[:, 3], 'b' + lineStyle)
        try:
            existingLegend = [txt._text for txt in axesR.get_legend().get_texts()]
        except AttributeError:
            existingLegend = []
        axesR.legend(existingLegend + 
                     [' '.join([label, 'r_w']), 
                      ' '.join([label, 'r_x']),
                      ' '.join([label, 'r_y']),
                      ' '.join([label, 'r_z']),])
        axesR.set_xlabel('Time (s)')
        axesR.set_ylabel('Quaternion components')
        axesR.set_ylim([-1, 1])
    if 'point' in inDict:
        if axesT is None:
            fig, axesT = plt.subplots()
        if kwargs.get('zeroT', False):
            point = np.copy(inDict['point'])
            firstPoint = inDict['point'][0, :]
            point = point - firstPoint
        else:
            point = inDict['point']
        axesT.plot(ts, point[:, 0], 'r' + lineStyle)
        axesT.plot(ts, point[:, 1], 'g' + lineStyle)
        axesT.plot(ts, point[:, 2], 'b' + lineStyle)
        try:
            existingLegend = [txt._text for txt in axesT.get_legend().get_texts()]
        except AttributeError:
            existingLegend = []
        axesT.legend(existingLegend + 
                     [' '.join([label, 'x']),
                      ' '.join([label, 'y']),
                      ' '.join([label, 'z']),])
        axesT.set_xlabel('Time (s)')
        axesT.set_ylabel('Coords (m)')
    callback = kwargs.get('callback')
    if callback is not None:
        del kwargs['callback']
        kwargs['axesT'] = axesT
        kwargs['axesR'] = axesR
        kwargs['minTime'] = minTime
        kwargs['maxTime'] = maxTime
        callback(**kwargs)

def plotPoseMultiChannel(inDict, **kwargs):
    if 'data' in inDict:
        inDict = inDict['data']
    axesR = kwargs.get('axesR')
    if axesR is None:
        fig, allAxes = plt.subplots(2, 1)
        fig.suptitle(kwargs.get('title', ''))
        kwargs['axesT'] = allAxes[0]
        kwargs['axesR'] = allAxes[1]
    bodyIds = kwargs.get('bodyIds', inDict.keys())
    include = kwargs.get('include', [])
    exclude = kwargs.get('exclude', [])
    lineStyle = itertools.cycle(('-', '--', '-.', ':'))
    for name in bodyIds:
        select_body = all([(inc in name) for inc in include]) and all([not (exc in name) for exc in exclude])
        if select_body:  # modify this line to plot whichever markers you want
            try:
                subDataDict = inDict[name]['pose6q']
            except KeyError:
                try:
                    subDataDict = inDict[name]['point3']
                except KeyError:
                    continue
            kwargs['label'] = name
            kwargs['lineStyle'] = next(lineStyle)
            plotPose(subDataDict, **kwargs)
            

"""
Plot the 3D trajectory of a body or marker for the entire duration of recording

To select which bodies to plot using bodyID: 
 - look at the bodyIDs parsed using importVicon
 - pass a list of strings present in the bodyID of choice, through the parameter include
 - pass a list of strings that should be absent from the bodyID of choice, through the parameter exclude
"""

# TODO: this function could be built up to become the core of pose3dMultiChannel
def plotPose3d(points, **kwargs):
    ax = plt.axes(projection='3d')
    X = points[0, :]
    Y = points[1, :]
    Z = points[2, :]
    ax.scatter3D(X, Y, Z, marker='o', s=0.5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

# legacy name for the above function
def plotPoints(points, **kwargs):
    plotPose3d(points, **kwargs)

def plotPose3dMultiChannel(dataDict, **kwargs):
    if 'data' in dataDict:
        dataDict = dataDict['data']
    ax = kwargs.get('ax')
    if ax is None:
        fig, ax = plt.subplots(projection='3d')
        kwargs['ax'] = ax
    bodyIds = kwargs.get('bodyIds', dataDict.keys())
    include = kwargs.get('include', [])
    exclude = kwargs.get('exclude', [])
    handles = []
    for name in bodyIds:
        select_body = all([(inc in name) for inc in include]) and all([not (exc in name) for exc in exclude])
        if select_body:  # modify this line to plot whichever markers you want
            marker_pose = dataDict[name]['pose6q']['point']
            X = marker_pose[:, 0]
            Y = marker_pose[:, 1]
            Z = marker_pose[:, 2]
            handles.append(ax.scatter3D(X, Y, Z, label=name))
            
            # Create cubic bounding box to simulate equal aspect ratio
            max_range = np.array([X.max() - X.min(), Y.max() - Y.min(), Z.max() - Z.min()]).max()
            Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (X.max() + X.min())
            Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (Y.max() + Y.min())
            Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (Z.max() + Z.min())
            # Comment or uncomment following both lines to test the fake bounding box:
            for xb, yb, zb in zip(Xb, Yb, Zb):
                ax.plot([xb], [yb], [zb], 'w')

    unit = kwargs.get('unit', '')
    ax.set_xlabel('X' + unit)
    ax.set_ylabel('Y' + unit)
    ax.set_zlabel('Z' + unit)
    if kwargs.get('legend') is not None:
        ax.legend(handles, kwargs.get('legend'))
    else:
        ax.legend()
    callback = kwargs.get('callback')
    if callback is not None:
        kwargs['axes'] = ax  # TODO: make this handling consistent across the library
        callback(**kwargs)

# legacy name for plotPose3d
def plotTrajectories(dataDict, bodyIds, include, exclude, **kwargs):
    kwargs['bodyIds'] = bodyIds
    kwargs['include'] = include
    kwargs['exclude'] = exclude 
    plotPose3dMultiChannel(dataDict, **kwargs)
    