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

VisualiserOpticFlow - for dense optic flow maps ...
"""

import numpy as np
import math

# Local imports
from .visualiserBase import Visualiser

# A function intended to find the nearest timestamp
# adapted from https://stackoverflow.com/questions/2566412/find-nearest-value-in-numpy-array
def findNearest(array, value):
    idx = np.searchsorted(array, value) # side="left" param is the default
    if idx > 0 and ( \
            idx == len(array) or \
            math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
        return idx-1
    else:
        return idx


class VisualiserOpticFlow(Visualiser):

    data_type = 'flowMap'

    def __init__(self, data):
        self.colorwheel = self.make_colorwheel()  # shape [55x3]
        self.set_data(data)

    def set_data(self, data):
        self.__data = {}
        self.__data.update(data)

    def get_colorfmt(self):
        return 'rgb'

    def get_default_image(self):
        x, y = self.get_dims()
        # Return an x,y,3 by default i.e. rgb, for safety, since in the absence of data we may not know how the texture's colorfmt is set
        return np.ones((x, y, 3), dtype=np.uint8) * 128 # TODO: Hardcoded midway (grey) value

    # TODO: There can be methods which better choose the best frame, or which create a visualisation which
    # respects the time_window parameter
    def get_frame(self, time, timeWindow, **kwargs):
        data = self.__data
        if time < data['ts'][0] - timeWindow / 2 or time > data['ts'][-1] + timeWindow / 2:
            # Gone off the end of the frame data
            image = self.get_default_image()
        else:
            frameIdx = findNearest(data['ts'], time)
            image = self.flow_to_color(data['flowMaps'][frameIdx])

        return image

    def get_dims(self):
        try:
            data = self.__data
        except AttributeError: # data hasn't been set yet
            return 1, 1
        x = data['dimX'] if 'dimX' in data else data['flowMaps'][0].shape[1]
        y = data['dimY'] if 'dimY' in data else data['flowMaps'][0].shape[0]
        return x, y

    ''' The methods make_colorwheel, flow_to_color and flow_uv_to_colors are taken from
    https://github.com/tomrunia/OpticalFlow_Visualization/blob/master/flow_vis/flow_vis.py'''
    @staticmethod
    def make_colorwheel():
        """
        Generates a color wheel for optical flow visualization as presented in:
            Baker et al. "A Database and Evaluation Methodology for Optical Flow" (ICCV, 2007)
            URL: http://vision.middlebury.edu/flow/flowEval-iccv07.pdf
        Code follows the original C++ source code of Daniel Scharstein.
        Code follows the the Matlab source code of Deqing Sun.
        Returns:
            np.ndarray: Color wheel
        """

        RY = 15
        YG = 6
        GC = 4
        CB = 11
        BM = 13
        MR = 6

        ncols = RY + YG + GC + CB + BM + MR
        colorwheel = np.zeros((ncols, 3))
        col = 0

        # RY
        colorwheel[0:RY, 0] = 255
        colorwheel[0:RY, 1] = np.floor(255 * np.arange(0, RY) / RY)
        col = col + RY
        # YG
        colorwheel[col:col + YG, 0] = 255 - np.floor(255 * np.arange(0, YG) / YG)
        colorwheel[col:col + YG, 1] = 255
        col = col + YG
        # GC
        colorwheel[col:col + GC, 1] = 255
        colorwheel[col:col + GC, 2] = np.floor(255 * np.arange(0, GC) / GC)
        col = col + GC
        # CB
        colorwheel[col:col + CB, 1] = 255 - np.floor(255 * np.arange(CB) / CB)
        colorwheel[col:col + CB, 2] = 255
        col = col + CB
        # BM
        colorwheel[col:col + BM, 2] = 255
        colorwheel[col:col + BM, 0] = np.floor(255 * np.arange(0, BM) / BM)
        col = col + BM
        # MR
        colorwheel[col:col + MR, 2] = 255 - np.floor(255 * np.arange(MR) / MR)
        colorwheel[col:col + MR, 0] = 255
        return colorwheel

    def flow_uv_to_colors(self, u, v, convert_to_bgr=False):
        """
        Applies the flow color wheel to (possibly clipped) flow components u and v.
        According to the C++ source code of Daniel Scharstein
        According to the Matlab source code of Deqing Sun
        Args:
            u (np.ndarray): Input horizontal flow of shape [H,W]
            v (np.ndarray): Input vertical flow of shape [H,W]
            convert_to_bgr (bool, optional): Convert output image to BGR. Defaults to False.
        Returns:
            np.ndarray: Flow visualization image of shape [H,W,3]
        """
        flow_image = np.zeros((u.shape[0], u.shape[1], 3), np.uint8)
        ncols = self.colorwheel.shape[0]
        rad = np.sqrt(np.square(u) + np.square(v))
        a = np.arctan2(-v, -u) / np.pi
        fk = (a + 1) / 2 * (ncols - 1)
        k0 = np.floor(fk).astype(np.int32)
        k1 = k0 + 1
        k1[k1 == ncols] = 0
        f = fk - k0
        for i in range(self.colorwheel.shape[1]):
            tmp = self.colorwheel[:, i]
            col0 = tmp[k0] / 255.0
            col1 = tmp[k1] / 255.0
            col = (1 - f) * col0 + f * col1
            idx = (rad <= 1)
            col[idx] = 1 - rad[idx] * (1 - col[idx])
            col[~idx] = col[~idx] * 0.75  # out of range
            # Note the 2-i => BGR instead of RGB
            ch_idx = 2 - i if convert_to_bgr else i
            flow_image[:, :, ch_idx] = np.floor(255 * col)
        return flow_image

    def flow_to_color(self, flow_uv, clip_flow=None, convert_to_bgr=False):
        """
        Expects a two dimensional flow image of shape.
        Args:
            flow_uv (np.ndarray): Flow UV image of shape [H,W,2]
            clip_flow (float, optional): Clip maximum of flow values. Defaults to None.
            convert_to_bgr (bool, optional): Convert output image to BGR. Defaults to False.
        Returns:
            np.ndarray: Flow visualization image of shape [H,W,3]
        """
        assert flow_uv.ndim == 3, 'input flow must have three dimensions'
        assert flow_uv.shape[2] == 2, 'input flow must have shape [H,W,2]'
        if clip_flow is not None:
            flow_uv = np.clip(flow_uv, 0, clip_flow)
        u = flow_uv[:, :, 0]
        v = flow_uv[:, :, 1]
        rad = np.sqrt(np.square(u) + np.square(v))
        rad_max = np.max(rad)
        epsilon = 1e-5
        u = u / (rad_max + epsilon)
        v = v / (rad_max + epsilon)
        return self.flow_uv_to_colors(u, v, convert_to_bgr)

