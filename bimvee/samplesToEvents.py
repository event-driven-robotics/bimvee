# -*- coding: utf-8 -*-
"""
Copyright (C) 2021 Event-driven Perception for Robotics
Authors: Simon Muller-Cleve
         Sim Bamford
         
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

the samplesToEvents function provides generic conversion from dense single- or
multi-channel data samples (which have typically but not necessarily been
sampled regularly in time) to polarised address events.

inDict comes in containing at least: 
'ts' - float64, assumed monotonically increasing
'value unless otherwise named by the kwarg 'valueKey' - a numpy array of samples
with one row (zeroth dimension) for each ts, and additional dimensions sufficient
to contain the address space. For example, if the input is from a camera,
there may be two additional dimensions for greyscale or 3 in case of rgb data.
Hereafter, a single pixel / taxel etc is referred to as a sensor.

The value data is assumed to be treated in the linear scale - i.e. threshold
applied linearly.
If instead relative thresholding is desired, then for the most general treatment,
the samples should be converted to the log domain prior to calling this function.
TODO: this could be internalised.

For each sensor, the following treatment should be performed.
Sample the first value. Move forward, applying any drift, until a threshold
crossing is found, taking into consideration any drift value.
Commit an event of the appropriate type at the appropriate linearly interpolated
time. Then move forward by the refractory period, before resampling the signal
at that moment. Repeat.
TODO: drift not yet implemented.

TODO: ...
    drift and refractory periods may be randomised by sensor.
    additive noise might be an option, perhaps following v2e, but need
    to think about generality of noise model.

Note: The code is somewhat inefficient in that it doesn't make use of array
operations across multiple sensors. On the other hand, it's probably a better
starting point for a Cuda implementation, providing only that we are genuinely
converting data in batch.
'''
"""

#%%

import numpy as np

from .split import mergeDataTypeDicts

def samplesToEvents(inDict, **kwargs):
    ts = inDict['ts']
    numSamples = len(ts)
    valueKey = kwargs.pop('valueKey', 'value')
    values = inDict[valueKey]
    assert values.shape[0] == numSamples
    ndims = values.ndim
    if ndims > 1:
        # Iterate through the first dimension of values
        results = []
        for addr in range(values.shape[1]):
            subValues = np.take(values, addr, axis=1)
            subValues = np.squeeze(subValues)
            subInDict = {'ts': ts,
                         'value': subValues}
            subOutDict = samplesToEvents(subInDict, **kwargs)
            numSubSamples = len(subOutDict['ts'])
            subAddrs = subOutDict.pop('addr', None)
            newAddrs = np.ones((numSubSamples), dtype = np.int32) * addr
            if subAddrs is None:
                subOutDict['addr'] = newAddrs
            else:
                subOutDict['addr'] = np.concatenate((newAddrs, subAddrs), axis=1)
            results.append(subOutDict)
    else:
        # We have a single 'sensor' - get on with the conversion
        # Unpack params
        thr_ON = kwargs.get('thresholdUp', kwargs.get('threshold', None))
        if thr_ON is None:
            raise NotImplementedError('Choose a method for setting the thresholds')
        thr_OFF = -kwargs.get('thresholdDown', thr_ON) #Note: should be defined positively in the first case
        refr_per = kwargs.get('refractoryPeriod', 0.0)
        # build lists for new events, before converting to arrays at the end
        t_event = ts[0] - refr_per
        last_val = values[0]
        ON_spikes = []
        OFF_spikes = []
        values = values.astype(np.float64) # TODO: consider if this is necessary
        delta_input = np.diff(values)
        dt = np.diff(ts)
        m = delta_input/dt
    
        # calc first event in new sample
        first_event_sample = False
        # create dummy to check if in sample or above
        t_event_dummy = None

        for i in range(len(delta_input)):
            # for the time between two samples
            # calc in between two samples
            in_sample = True
    
            while in_sample:
                # calculate after change from one sample to the next
                if first_event_sample:
                    # ON event
                    if delta_input[i] > 0:
                        # thr reached after refr. per.
                        if ((values[i]+m[i]*(t_event + refr_per - ts[i])) - 
                            last_val >= thr_ON):
                            t_event_dummy = t_event + refr_per
                            # check if t_event in sample or above
                            if t_event_dummy > ts[i + 1]:
                                in_sample = False
                                first_event_sample = True
                            else:
                                t_event = t_event_dummy
                                last_val = values[i] + m[i]*(t_event - ts[i])
                                ON_spikes.append(t_event)
                                first_event_sample = False
                        # after refe. per. thr. not reached
                        else:
                            t_event_dummy = (((last_val + thr_ON - values[i])
                                              / m[i]) + ts[i])
                            # check if t_event in sample or above
                            if t_event_dummy > ts[i + 1]:
                                in_sample = False
                                first_event_sample = True
                            else:
                                last_val = last_val + thr_ON
                                t_event = t_event_dummy
                                ON_spikes.append(t_event)
                                first_event_sample = False
                    # OFF event
                    elif delta_input[i] < 0:
                        # thr reached after refr. per.
                        if ((values[i] + m[i] * (t_event + refr_per - ts[i]))
                           -last_val <= thr_OFF):
                            t_event_dummy = t_event + refr_per
                            # check if t_event in sample or above
                            if t_event_dummy > ts[i+1]:
                                in_sample = False
                                first_event_sample = True
                            else:
                                t_event = t_event_dummy
                                last_val = values[i] + m[i] * (t_event - ts[i])
                                OFF_spikes.append(t_event)
                                first_event_sample = False
                        # after refe. per. thr. not reached
                        else:
                            t_event_dummy = (((last_val + thr_OFF - values[i])
                                             / m[i]) + ts[i])
                            # check if t_event in sample or above
                            if t_event_dummy > ts[i+1]:
                                in_sample = False
                                first_event_sample = True
                            else:
                                last_val = last_val + thr_OFF
                                t_event = t_event_dummy
                                OFF_spikes.append(t_event)
                                first_event_sample = False
                    # no change in this sample
                    else:
                        in_sample = False
                        first_event_sample = True
                # calculate events in sample
                else:
                    # ON events
                    if delta_input[i] > 0:
                        # thr. reached after ref. per.
                        if refr_per*m[i] >= thr_ON:
                            t_event_dummy = t_event + refr_per
                            # check if t_event in sample or above
                            if t_event_dummy > ts[i+1]:
                                in_sample = False
                                first_event_sample = True
                            else:
                                t_event = t_event_dummy
                                last_val = refr_per*m[i] + last_val
                                ON_spikes.append(t_event)
                        # thr. not reached after ref.
                        else:
                            t_event_dummy = (thr_ON/m[i]) + t_event
                            # check if event is in sample or above
                            if t_event_dummy > ts[i+1]:
                                in_sample = False
                                first_event_sample = True
                            else:
                                t_event = t_event_dummy
                                last_val = last_val + thr_ON
                                ON_spikes.append(t_event)
                    # OFF events
                    elif delta_input[i] < 0:
                        # thr. reached after ref. per.
                        if refr_per*m[i] <= thr_OFF:
                            t_event_dummy = t_event + refr_per
                            # check if t_event in sample or above
                            if t_event_dummy > ts[i+1]:
                                in_sample = False
                                first_event_sample = True
                            else:
                                t_event = t_event_dummy
                                last_val = refr_per*m[i] + last_val
                                OFF_spikes.append(t_event)
                        # thr. not reached after ref.
                        else:
                            t_event_dummy = (thr_OFF/m[i]) + t_event
                            # check if event is in sample or above
                            if t_event_dummy > ts[i+1]:
                                in_sample = False
                                first_event_sample = True
                            else:
                                t_event = t_event_dummy
                                last_val = last_val + thr_OFF
                                OFF_spikes.append(t_event)
                    # no change in this sample
                    else:
                        in_sample = False
                        # TODO double check here!
                        first_event_sample = True
        # Now we have lists of ON and OFF times - convert to a single sorted
        #dataType dict, including a 'pol' field
        ON_spikes = np.array(ON_spikes, dtype = np.float64)
        onDict = {'ts': ON_spikes,
                  'pol': np.ones_like(ON_spikes, dtype=np.bool)
                  }
        OFF_spikes = np.array(OFF_spikes, dtype = np.float64)
        offDict = {'ts': OFF_spikes,
                  'pol': np.zeros_like(OFF_spikes, dtype=np.bool)
                  }
        results = [onDict, offDict]
    return mergeDataTypeDicts(results) # This will sort by default

#%%

