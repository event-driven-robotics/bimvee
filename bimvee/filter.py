# -*- coding: utf-8 -*-
"""
Created on Fri Jul  5 14:23:08 2024

@author: sbamford
"""

"""
also known as a salt-and-pepper filter,
lets an event pass if there has been a previous event 
within a certain spatio-temporal window.
"""
def filter_spatiotemporal(events, time_window=0.05, neighbourhood=1):
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

#TODO: make a function that will descend a hierarchy of containers 
# and apply to all dvs datatypes
