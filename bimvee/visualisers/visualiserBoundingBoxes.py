# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
        Massimiliano Iacono

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
Functionality for serving images which represent the data at a certain time
(or given a certain time window).
The intended use case is to support video-like playback of data
There is a generic Visualiser class - it contains a dataType dict.
This is subclassed for each supported dataType (e.g. dvs, frame, pose etc)
Each subclass should implement basic methods: get_dims, get_frame, get_colorfmt
Implementing set_data allows the visualiser to do some preparation for visualisation
when it receives new data.

VisualiserBoundingBoxes doesn't actually create a visualisation but rather
queries the data for the bounding boxes within the time window and passes
these out for visualisation as an overlay.
"""

import numpy as np

# Local imports
from .visualiserBase import Visualiser


class VisualiserBoundingBoxes(Visualiser):

    data_type = 'boundingBoxes'

    def __init__(self, data):
        self.set_data(data)

    def set_data(self, data):
        self.__data = {}
        self.__data.update(data)

    def get_data(self):
        return self.__data

    def get_frame(self, time, timeWindow, **kwargs):
        if self.__data is None or not kwargs.get('show_bounding_boxes', True):
            return [[0, 0, 0, 0]]
        gt_bb = self.__data
        indices = abs(gt_bb['ts'] - time) < timeWindow
        if not any(indices):
            if not kwargs.get('interpolate'):
                return [[0, 0, 0, 0]]
        if kwargs.get('interpolate'):
            boxes = []
            for label in np.unique(gt_bb['label']):
                label_mask = gt_bb['label'] == label
                ts = gt_bb['ts'][label_mask]
                minY = gt_bb['minY'][label_mask]
                minX = gt_bb['minX'][label_mask]
                maxY = gt_bb['maxY'][label_mask]
                maxX = gt_bb['maxX'][label_mask]

                i1 = np.searchsorted(ts, time)
                i0 = i1 - 1
                if i0 < 0:
                    if abs(ts[0] - time) < timeWindow:
                        if kwargs.get('with_labels', True) and 'label' in gt_bb.keys():
                            boxes.append((minY[0], minX[0], maxY[0], maxX[0], label))
                        else:
                            boxes.append((minY[0], minX[0], maxY[0], maxX[0]))
                        continue
                    else:
                        continue
                if i1 >= len(ts):
                    if abs(ts[-1] - time) < timeWindow:
                        if kwargs.get('with_labels', True) and 'label' in gt_bb.keys():
                            boxes.append((minY[-1], minX[-1], maxY[-1], maxX[-1], label))
                        else:
                            boxes.append((minY[-1], minX[-1], maxY[-1], maxX[-1]))
                        continue
                    else:
                        continue

                minY_interp = minY[i0] + ((minY[i1] - minY[i0]) / abs(ts[i1] - ts[i0])) * (time - ts[i0])
                minX_interp = minX[i0] + ((minX[i1] - minX[i0]) / abs(ts[i1] - ts[i0])) * (time - ts[i0])
                maxY_interp = maxY[i0] + ((maxY[i1] - maxY[i0]) / abs(ts[i1] - ts[i0])) * (time - ts[i0])
                maxX_interp = maxX[i0] + ((maxX[i1] - maxX[i0]) / abs(ts[i1] - ts[i0])) * (time - ts[i0])
                if kwargs.get('with_labels', True) and 'label' in gt_bb.keys():
                    boxes.append((minY_interp, minX_interp, maxY_interp, maxX_interp, label))
                else:
                    boxes.append((minY_interp, minX_interp, maxY_interp, maxX_interp))
            boxes = np.array(boxes).astype(int)
        else:
            boxes = np.column_stack((gt_bb['minY'][indices], gt_bb['minX'][indices],
                                     gt_bb['maxY'][indices], gt_bb['maxX'][indices])).astype(np.int)
            if kwargs.get('with_labels', True) and 'label' in gt_bb.keys():
                labels = gt_bb['label'][indices].astype(np.int)
                boxes = np.column_stack([boxes, labels])
        boxes = np.unique(boxes, axis=0)
        return boxes

    def get_settings(self):
        settings = {'with_labels': {'type': 'boolean',
                                    'default': True
                                    },
                    'show_bounding_boxes': {'type': 'boolean',
                                            'default': True
                                            },
                    'interpolate': {'type': 'boolean',
                                    'default': False
                                    }
                    }
        return settings
