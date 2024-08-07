# -*- coding: utf-8 -*-
'''
Copyright (C) 2024 Event-driven Perception for Robotics
Authors: Sim Bamford, Mohammadreza Koolani
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)

filter_spatiotemporal:
Also known as a salt-and-pepper filter,
lets an event pass if there has been a previous event 
within a certain spatio-temporal window.
'''

import numpy as np

def filter_spatiotemporal_single(events, time_window=0.05, neighbourhood=1):
    xs = events['x']
    ys = events['y']
    ts = events['ts']
    pol = events['pol']
 
    offset_neg = neighbourhood
    offset_pos = neighbourhood + 1
    
    xs = xs + neighbourhood
    ys = ys + neighbourhood
    keep = np.zeros_like(ts, dtype=bool)
    last_ts = np.full((np.max(ys) + neighbourhood * 2, 
                       np.max(xs) +  + neighbourhood * 2), 
                      -np.inf)
    for i, (x, y, t) in enumerate(zip(xs, ys, ts)):
        if t - last_ts[y, x] <= time_window:
            keep[i] = True
        last_ts[y - offset_neg : y + offset_pos, 
                x - offset_neg : x + offset_pos] = t #update
    return {
        'x' : events['x'][keep],
        'y' : events['y'][keep],
        'ts' : events['ts'][keep],
        'pol' : events['pol'][keep],
        }

def filter_spatiotemporal(in_dict, **kwargs):
    # check to see if this is dvs type:
    if (in_dict.get('data_type', '') == 'dvs'
        or ('x' in in_dict.keys()
            and 'y' in in_dict.keys()
            and 'pol' in in_dict.keys()
            and 'ts' in in_dict.keys())):
        return filter_spatiotemporal_single(in_dict, **kwargs)
    else:
        new_dict = {}
        for key, value in in_dict.items():
            if key == 'dvs':
                new_dict[key] = filter_spatiotemporal_single(value, **kwargs)
            elif type(value) == dict:
                new_dict[key] = filter_spatiotemporal(value, **kwargs)
            else:
                new_dict[key] = value
        return new_dict
