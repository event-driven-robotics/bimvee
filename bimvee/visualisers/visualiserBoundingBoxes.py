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

    def get_frame(self, time, timeWindow, **kwargs):
        if self.__data is None:
            return [[0, 0, 0, 0]]
        gt_bb = self.__data
        box_index = np.searchsorted(gt_bb['ts'], time)
        if box_index == gt_bb['ts'].shape[0] or abs(gt_bb['ts'][box_index] - time) > timeWindow:
            return [[0, 0, 0, 0]]
        indices = gt_bb['ts'] == gt_bb['ts'][box_index]
        boxes = np.column_stack((gt_bb['minY'][indices], gt_bb['minX'][indices],
                                 gt_bb['maxY'][indices], gt_bb['maxX'][indices])).astype(np.int)
        if kwargs['with_labels'] and 'label' in gt_bb.keys():
            labels = gt_bb['label'][indices].astype(np.int)
            boxes = np.column_stack([boxes, labels])

        return boxes