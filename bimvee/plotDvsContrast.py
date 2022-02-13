# -*- coding: utf-8 -*-
'''
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
plotDvsContrast takes 'inDict' - a dict containing an imported ae file, 
as created by importAe, and creates a series of green/red plots of
polarity data.
It creates a completely separate plot for each channel which contains 'pol' data
Creates an image from events with contrast, by accumulating events up and down 
to a maximum level at which full color is used. 
The number of subplots is given by the numPlots parameter.
'distributeBy' can either be 'time' or 'events', to decide how the points 
around which data is rendered are chosen.
The events are then recruited by the time points, spreading out until
either they are about to overlap with a neighbouring point, or until 
a certain ratio of a full array is reached. 

Parameters which can be used:
 - numPlots
 - distributeBy
 - minTime, startTime
 - maxTime, endTime, stopTime
 - proportionOfPixels
 - contrast
 - flipVertical
 - flipHorizontal
 - transpose
'''

import numpy as np
import matplotlib.pyplot as plt
from math import log10, floor

from .split import selectByRange


def roundToSf(x, sig=3):
    try:
        return round(x, sig - int(floor(log10(abs(x)))) - 1)
    except ValueError:  # log of zero
        return 0


'''
nomenclature:
    idx = index
    ids = indices
'''


def idsEventsInTimeRange(events, **kwargs):
    startTime = kwargs.get('startTime', kwargs.get('minTime', kwargs.get('beginTime', events['ts'][0])))
    endTime = kwargs.get('stopTime', kwargs.get('maxTime', kwargs.get('endTime', events['ts'][-1])))
    # The following returns logical indices
    # return (events['ts'] >= startTime) & (events['ts'] < endTime)
    # Alternatively, search for the start and end indices, then return a range
    # This might be faster, given that the ts array is already sorted
    startIdx = np.searchsorted(events['ts'], startTime)
    endIdx = np.searchsorted(events['ts'], endTime)
    return range(startIdx, endIdx)


def getEventsInTimeRange(events, **kwargs):
    ids = kwargs.get('ids', idsEventsInTimeRange(events, **kwargs))
    return {
        'y': events['y'][ids],
        'x': events['x'][ids],
        'pol': events['pol'][ids],
        'ts': events['ts'][ids]
    }


def getEventImage(events, **kwargs):
    # dims might be in the events dict, but allow override from kwargs
    try:
        dimX = kwargs.get('dimX', events.get('dimX', np.max(events['x']) + 1))
        dimY = kwargs.get('dimY', events.get('dimY', np.max(events['y']) + 1))
    except ValueError:  # no defined dims and events arrays are empty
        dimX = 1
        dimY = 1
    try:
        if kwargs.get('image_type') == 'count':
            eventImagePos = np.histogram2d(events['y'][events['pol']],
                                         events['x'][events['pol']],
                                         bins=[dimY, dimX],
                                         range=[[-0.5, dimY-0.5], [-0.5, dimX-0.5]]
                                         )[0]
            eventImageNeg = np.histogram2d(events['y'][~events['pol']],
                                         events['x'][~events['pol']],
                                         bins=[dimY, dimX],
                                         range=[[-0.5, dimY-0.5], [-0.5, dimX-0.5]]
                                         )[0]
            if kwargs.get('pol_to_show') is None or kwargs.get('pol_to_show') == 'Both':
                eventImage = eventImagePos - eventImageNeg
            elif kwargs.get('pol_to_show') == 'Pos':
                eventImage = eventImagePos
            elif kwargs.get('pol_to_show') == 'Neg':
                eventImage = - eventImageNeg
        elif kwargs.get('image_type') == 'not_polarized':
            eventImage = np.histogram2d(events['y'],
                                        events['x'],
                                        bins=[dimY, dimX],
                                        range=[[-0.5, dimY-0.5], [-0.5, dimX-0.5]]
                                        )[0]
        elif kwargs.get('image_type') == 'time_image':
            eventImage = np.zeros((dimY, dimX))
            eventImage[events['y'], events['x']] = (events['ts'] - events['ts'][0])
            eventImage = eventImage / eventImage.max()
        elif kwargs.get('image_type') == 'binary':
            eventImage = np.zeros((dimY, dimX))
            if kwargs.get('pol_to_show') is None or kwargs.get('pol_to_show') == 'Both':
                eventImage[events['y'], events['x']] = (events['pol'].astype(int) * 2 - 1)
            elif kwargs.get('pol_to_show') == 'Pos':
                eventImage[events['y'][events['pol']], events['x'][events['pol']]] = 1
            elif kwargs.get('pol_to_show') == 'Neg':
                eventImage[events['y'][~events['pol']], events['x'][~events['pol']]] = -1
        elif kwargs.get('image_type') == 'coloured':
            pos_colour = [255, 0, 0]
            neg_colour = [0, 0, 255]
            eventImage = np.full((dimY, dimX, 3), 255, dtype=np.uint8)
            if kwargs.get('pol_to_show') is None or kwargs.get('pol_to_show') == 'Both':
                eventImage[events['y'], events['x']] = [pos_colour if p else neg_colour for p in events['pol']]
            elif kwargs.get('pol_to_show') == 'Pos':
                eventImage[events['y'][events['pol']], events['x'][events['pol']]] = pos_colour
            elif kwargs.get('pol_to_show') == 'Neg':
                eventImage[events['y'][~events['pol']], events['x'][~events['pol']]] = neg_colour
    except ValueError:
        pass
    except IndexError:
        pass
    # Clip the values according to the contrast
    if not kwargs.get('image_type') == 'coloured':
        contrast = kwargs.get('contrast', 3)
        eventImage = np.clip(eventImage, -contrast, contrast)
    return eventImage


def getEventImageForTimeRange(events, **kwargs):
    events = getEventsInTimeRange(events, **kwargs)
    return getEventImage(events, **kwargs)


'''
This function accepts a dict of events and returns a new event dict
formed by collecting count number of events in the past/future from time ts
direction:
  -1: look back in the past;
  0: look equally to past and future;
  1: look to the future
'''
def getEventsByCount(events, **kwargs):
    direction = kwargs.get('direction', -1)  # default direction is past
    ts = kwargs.get('ts', 0)
    count = kwargs.get('count', 0)
    seedEventId = np.searchsorted(events['ts'], ts, side='right')
    if direction == 0:
        halfCount = int(count/2)
        firstEventId = max(seedEventId - halfCount, 0)
        lastEventId = min(seedEventId + halfCount, len(events['ts']) - 1)
        # startOrEndTime reports end time in this case
        startOrEndTime = events['ts'][lastEventId]
    else:
        startOrEndEventId = min(max(seedEventId + direction * count, 0), 
                                len(events['ts']) - 1) # limit ID within range
        startOrEndTime = events['ts'][startOrEndEventId]
        firstEventId = min(seedEventId, startOrEndEventId)
        lastEventId = max(seedEventId, startOrEndEventId)
    selectedEvents = selectByRange(events, firstEventId, lastEventId + 1)
    return selectedEvents, startOrEndTime


'''
This function accepts a dict of events and returns an event-image
formed by collecting count number of events in the past/future from time ts
direction:
  -1: look back in the past;
  0: look equally to past and future;
  1: look to the future
'''
def getEventImageByCount(events, **kwargs):
    selectedEvents, startOrEndTime = getEventsByCount(events, **kwargs)
    return getEventImage(selectedEvents, **kwargs), startOrEndTime


def plotDvsContrastSingle(inDict, **kwargs):
    frameFromEvents = getEventImage(inDict, **kwargs)
    if kwargs.get('transpose', False):
        frameFromEvents = np.transpose(frameFromEvents)
    if kwargs.get('flipVertical', False):
        frameFromEvents = np.flip(frameFromEvents, axis=0)
    if kwargs.get('flipHorizontal', False):
        frameFromEvents = np.flip(frameFromEvents, axis=1)
    contrast = kwargs.get('contrast', 3)
    axes = kwargs.get('axes')
    if axes is None:
        fig, axes = plt.subplots()
    if kwargs.get('polarised', (kwargs.get('polarized'), False)):
        cmap = kwargs.get('cmap', kwargs.get('colormap', 'seismic_r'))
        image = axes.imshow(frameFromEvents, cmap=cmap,
                            vmin=-contrast, vmax=contrast)
    else:
        cmap = kwargs.get('cmap', kwargs.get('colormap', 'gray'))
        image = axes.imshow(frameFromEvents, cmap=cmap,
                            vmin=0, vmax=contrast)
    axes.set_aspect('equal', adjustable='box')
    title = kwargs.get('title')
    if title is not None:
        axes.set_title(title)

    callback = kwargs.get('callback')
    if callback is not None:
        kwargs['axes'] = axes
        callback(frameFromEvents=frameFromEvents, **kwargs)
    return image


def plotDvsContrast(inDicts, **kwargs):
    if isinstance(inDicts, list):
        for inDict in inDicts:
            plotDvsContrast(inDict, **kwargs)
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
                print('plotDvsContrast was called for file ' + fileName)
                title = (title + ' ' + fileName).lstrip()
        for key in inDict.keys():
            kwargs['title'] = (title + ' ' + key).lstrip()
            plotDvsContrast(inDict[key], **kwargs)
        return
    # From this point onwards, it's a data-type container
    if 'pol' not in inDict:
        return
    # From this point onwards, it's a dvs container        

    # The proportion of an array-full of events which is shown on a plot
    proportionOfPixels = kwargs.get('proportionOfPixels', 0.1)
    # useAllData overrides the above - the windows used for the images together include all the data 
    useAllData = kwargs.get('useAllData', False)

    numPlots = kwargs.get('numPlots', 6)

    # Choice of distributing by 'time' or by 'numEvents'
    distributeBy = kwargs.get('distributeBy', 'time').lower()

    # % Distribute plots in a raster with a 3:4 ratio
    numPlotsX = int(round(np.sqrt(numPlots / 3 * 4)))
    numPlotsY = int(np.ceil(numPlots / numPlotsX))

    # TODO: if the actual sensor size is known, use this instead of the following defaults
    minX = kwargs.get('minX', inDict['x'].min())
    maxX = kwargs.get('maxX', inDict['x'].max())
    minY = kwargs.get('minY', inDict['y'].min())
    maxY = kwargs.get('maxY', inDict['y'].max())
    kwargs['minX'] = minX
    kwargs['maxX'] = maxX
    kwargs['minY'] = minY
    kwargs['maxY'] = maxY

    numPixelsInArray = (maxY + 1 - minY) * (maxX + 1 - minX)
    numEventsToSelectEachWay = int(round(numPixelsInArray * proportionOfPixels / 2.0))

    # The following section results in a set of tuples of first and last event idx
    # one for each plot. It does this considering the choices of distributeBy, useAllData, and proportionOfPixels

    # unpack ts for brevity
    ts = inDict['ts']

    minTime = kwargs.get('minTime', kwargs.get('startTime', kwargs.get('beginTime', ts.min())))
    maxTime = kwargs.get('maxTime', kwargs.get('stopTime', kwargs.get('endTime', ts.max())))
    minEventIdx = np.searchsorted(ts, minTime)
    maxEventIdx = np.searchsorted(ts, maxTime)
    numEvents = maxEventIdx - minEventIdx
    if distributeBy == 'time':
        totalTime = maxTime - minTime
        timeStep = totalTime / numPlots
        if useAllData:
            timeBoundaries = np.arange(minTime, maxTime + timeStep / 2, timeStep)
            firstEventIds = [
                np.where(ts >= timeBoundary)[0][0]
                for timeBoundary in timeBoundaries]
            lastEventIds = [firstEventIdx - 1 for firstEventIdx in firstEventIds]
            lastEventIds = lastEventIds[1:]
            firstEventIds = firstEventIds[:-1]
        else:
            timeCentres = np.arange(minTime + timeStep * 0.5, maxTime, timeStep)
            centreEventIds = np.searchsorted(ts, timeCentres)
            firstEventIds = [idx - numEventsToSelectEachWay for idx in centreEventIds]
            lastEventIds = [idx + numEventsToSelectEachWay for idx in centreEventIds]
    else:  # distribute by event number
        eventsPerStep = int(numEvents / numPlots)
        if useAllData:
            firstEventIds = range(minEventIdx, maxEventIdx, eventsPerStep)
            lastEventIds = [firstEventIdx - 1 for firstEventIdx in firstEventIds]
            lastEventIds = lastEventIds[1:]
            firstEventIds = firstEventIds[:-1]
        else:
            centreEventIds = range(int(eventsPerStep / 2), numEvents, eventsPerStep)
            firstEventIds = [idx - numEventsToSelectEachWay for idx in centreEventIds]
            lastEventIds = [idx + numEventsToSelectEachWay for idx in centreEventIds]
    firstEventIds = np.clip(firstEventIds, minEventIdx, maxEventIdx - 1)
    lastEventIds = np.clip(lastEventIds, minEventIdx, maxEventIdx - 1)
    firstTimes = ts[firstEventIds]
    lastTimes = ts[lastEventIds]
    timeCentres = (firstTimes + lastTimes) / 2
    titles = [
        str(roundToSf(firstTime)) + ' - ' + str(roundToSf(lastTime)) + ' s'
        for firstTime, lastTime in zip(firstTimes, lastTimes)]

    fig, allAxes = plt.subplots(numPlotsY, numPlotsX)
    if numPlots == 1:
        allAxes = [allAxes]
    else:
        allAxes = allAxes.flatten()
    fig.suptitle(kwargs.get('title', ''))

    for axes, firstEventIdx, lastEventIdx, title in zip(allAxes, firstEventIds, lastEventIds, titles):
        dvsDataDict = {
            'x': inDict['x'][firstEventIdx:lastEventIdx],
            'y': inDict['y'][firstEventIdx:lastEventIdx],
            'pol': inDict['pol'][firstEventIdx:lastEventIdx],
            'ts': inDict['ts'][firstEventIdx:lastEventIdx]
        }
        kwargs['title'] = title
        image = plotDvsContrastSingle(inDict=dvsDataDict, axes=axes, **kwargs)
    fig.colorbar(image)
    return timeCentres
