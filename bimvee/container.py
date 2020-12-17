# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
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

A Container class which encapsulates some basic container-level manipulations.

Firstly, the loosely enforced 3-tier hierarchy of data-channel-datatype within 
an imported dict has the advantage that channels can help organise data but 
it has the disadvantage that when there is just a single dict for each datatype, 
the user may not know or care about the channel names, yet needs them in order
to navigate the data. 
Therefore the getDataType() method allows the user to get the dict for a chosen
dataType providing only that there's a single example. 
"""

class Container():
    
    def __init__(self, container):
       self.container = container
     
    # This code assumes the 3-level hierarchy    
    def getDataType(self, dataType):
        dataTypeDicts = []
        for channelKey in self.container['data'].keys():
            for dataTypeKey in self.container['data'][channelKey]:
                if dataTypeKey == dataType:
                    dataTypeDicts.append(self.container['data'][channelKey][dataTypeKey])
        if len(dataTypeDicts) == 0:
            raise ValueError('No dicts found for dataType ' + dataType)
        if len(dataTypeDicts) > 1:
            raise ValueError('More than one dict found for dataType: ' + dataType)        
        return dataTypeDicts[0]

    def getAllDataOfType(self, dataType):
        dataTypeDicts = {}
        for channelKey in self.container['data'].keys():
            for dataTypeKey in self.container['data'][channelKey]:
                if dataTypeKey == dataType:
                    dataTypeDicts[channelKey] = self.container['data'][channelKey][dataTypeKey]
        return dataTypeDicts

