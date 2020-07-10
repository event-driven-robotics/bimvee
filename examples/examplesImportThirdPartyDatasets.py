# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
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
This script contains a set of examples of how to use the import functions of 
the bimvee library.
In each case, change the file paths as required to point to your own data.
"""

#%% import from intel realsense rosbag dump

from bimvee.importRpgDvsRos import importRpgDvsRos

filePathOrName = '/path/to/data.bag'

# You can just import straight away like this:
container = importRpgDvsRos(filePathOrName=filePathOrName)

# Alternatively, you can define a template and get just the topics you want
template = {
    'pose': {
            'pose6q': '/device_0/sensor_0/Pose_0/pose/transform/data',
            },
    'left': {
            'frame': '/device_0/sensor_0/Fisheye_1/image/data',
            'cam': '/device_0/sensor_0/Fisheye_1/info/camera_info',
            },
    'right': {
            'frame': '/device_0/sensor_0/Fisheye_2/image/data',
            'cam': '/device_0/sensor_0/Fisheye_2/info/camera_info',
            }
        }
container = importRpgDvsRos(filePathOrName=filePathOrName, template=template)

#%% Import the MVSEC dataset

from bimvee.importRpgDvsRos import importRpgDvsRos

'''
The dataset is available here:
https://daniilidis-group.github.io/mvsec/download/
First download the rosbag files.
'''
# This is for the first data file
filePathOrName = '/path/to/indoor_flying1_data.bag'
filePathOrName = 'C:/data/mvsec/indoor_flying1_data.bag'
template = {
    'davisLeft': {
        'dvs': '/davis/left/events',
        'frame': '/davis/left/image_raw',
        'imu': '/davis/left/imu',
        'cam': '/davis/left/camera_info',
    }, 'davisRight': {
        'dvs': '/davis/right/events',
        'frame': '/davis/right/image_raw',
        'imu': '/davis/right/imu',
        'cam': '/davis/right/camera_info',
        }
    }
containerData = importRpgDvsRos(filePathOrName=filePathOrName, template=template)

#%%
# This is for the corresponding ground-truth file 
filePathOrName = '/path/to/indoor_flying1_gt.bag'
filePathOrName = 'C:/data/external/mvsec/indoor_flying1_gt.bag'
template = {
    'poseLocal': {
        'pose6q': '/davis/left/odometry',
    }, 'poseGlobal': {
        'pose6q': '/davis/left/pose',
    }, 'depthLeft': {
        'frame': '/davis/left/depth_image_raw',
    }, 'depthRight': {
        'frame': '/davis/right/depth_image_raw',
        }
    }
#containerGt = importRpgDvsRos(filePathOrName=filePathOrName, template=template)
containerGt = importRpgDvsRos(filePathOrName=filePathOrName)

# If you want, you can combine these containers into a single container;
# In this case, the 'info' branch becomes inconsistent, but it doesn't really matter


container = containerData
container['data'].update(containerGt['data'])



