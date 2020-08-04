#%%
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: adinale

-------------
 MAIN STEPS:
-------------
1. Load StEFI (S), Vicon (V) and RealSense (RS) data dumped during the same dataset collection
2. Align RealSense data to StEFI and Vicon data TEMPORALLY
3. Interpolate RealSense and Vicon poses over a common time window
4. Find through optimization the rigid transformation for aligning RealSense poses to Vicon poses (SPATIAL alignment)
5. ...

This script has been written taking some parts from:
- TODO: add 
"""

import os, sys

# Get os-specific path for local libraries (YMMV)
prefix = 'C:/' if os.name == 'nt' else '/home/adinale/'

# Add path to bimvee library
sys.path.append(os.path.join(prefix, 'src/bimvee/'))
# Add path to mustard
sys.path.append(os.path.join(prefix, 'src/mustard'))

#%% Load StEFI and Vicon data

from bimvee.importIitYarp import importIitYarp

datasetPath = "Documents/Event-Driven_Dataset"
datasetName = "chessboardViconRealSense"
trialName = "2020-06-26_trial01"

filePathOrName = os.path.join(prefix, datasetPath + "/" + datasetName + "/" + trialName)
containerYarp = importIitYarp(filePathOrName=filePathOrName, tsBits=30)

#%% Load RealSense data (ONLY poses)

from bimvee.importRpgDvsRos import importRpgDvsRos

trialNameRS = "2020-06-26_trial01/RealSense/trial01.bag"
bagname = os.path.join(prefix, datasetPath + "/" + datasetName + "/" + trialNameRS)
containerRealsense = importRpgDvsRos(filePathOrName=bagname, zeroTimestamps=True)

#%% Time alignment of RealSense data to StEFI and Vicon data
import pyrealsense2 as rs

# Create pipeline
pipeline = rs.pipeline()

# Create a config object
config = rs.config()

# Tell config that we will use a recorded device from filem to be used by the pipeline through playback.
rs.config.enable_device_from_file(config, bagname)

# Configure the pipeline streams (pose and fisheye cameras)
config.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipeline.start(config)

try:
    for _ in range(1):
        # Wait for the next set of frames from the camera
        frames = pipeline.wait_for_frames()

        containerRealsense['info']['tsOffsetFromInfo'] = frames.get_timestamp()/1000 # microseconds to milliseconds
finally:
    pipeline.stop()

# Compute and apply the temporal offset to the RealSense data
offset = containerRealsense['info']['tsOffsetFromInfo'] - \
    min(containerRealsense['info']['tsOffsetFromInfo'], containerYarp[0]['info']['tsOffsetFromInfo'], containerYarp[1]['info']['tsOffsetFromInfo'])

from bimvee.timestamps import offsetTimestampsForAContainer

offsetTimestampsForAContainer(containerRealsense, offset)

#%% Create a new container for Vicon and RealSense data
import copy
import numpy as np
from bimvee.importIitVicon import separateMarkersFromSegments
from bimvee.split import splitByLabel
from bimvee.plot import plot

container = {'info': copy.deepcopy(containerYarp[1]['info']),
             'data': {'vicon': copy.deepcopy(containerYarp[1]['data']['vicon']),
                      'rs': copy.deepcopy(containerRealsense['data']['/device_0/sensor_0/Pose_0/pose/transform/data'])}}

# Separate markers (pose3) from segments (pose6q)
container['data']['vicon']['pose6q'] = separateMarkersFromSegments(container['data']['vicon']['pose6q'])['pose6q']

# Remove null poses
selected = np.any(container['data']['vicon']['pose6q']['point'] == np.zeros((1, 3)), axis = 1)
for key in container['data']['vicon']['pose6q'].keys():
    container['data']['vicon']['pose6q'][key] = container['data']['vicon']['pose6q'][key][~selected]

# Select only poses representing STEFI
container['data']['vicon']['pose6q'] =  splitByLabel(container['data']['vicon']['pose6q'], 'bodyId')['Subj_StEFI::Seg_StEFI']

# Remove repeated poses
_, selected = np.unique(container['data']['vicon']['pose6q']['ts'], return_index=True)
for key in container['data']['vicon']['pose6q'].keys(): 
    container['data']['vicon']['pose6q'][key] = container['data']['vicon']['pose6q'][key][selected]

# Clean-up
del selected, key

# Plot the obtained container
%matplotlib auto
plot(container)

#%% Interpolate with Slerp the Vicon and RealSense rotations
from bimvee.split import cropTime
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation

# Find the first common time and the last common time
startTs = max(container['data']['vicon']['pose6q']['ts'][0], container['data']['rs']['pose6q']['ts'][0])
stopTs = min(container['data']['vicon']['pose6q']['ts'][-1], container['data']['rs']['pose6q']['ts'][-1])

# Crop data to have ROUGHLY the same common time window
container['data']['vicon']['pose6q'] = cropTime(container['data']['vicon']['pose6q'], startTime=startTs, stopTime=stopTs, zeroTime=False)
container['data']['rs']['pose6q'] = cropTime(container['data']['rs']['pose6q'], startTime=startTs, stopTime=stopTs, zeroTime=False)

# Use the 'xyzw' format for quaternion
q2r_order = [1, 2, 3, 0]
# Use the 'wxyz' format for quaternion
r2q_order = [3, 0, 1, 2]

# Guarantee a feasible time window for the interpolation (i.e. it must be within the limits of both Vicon and RealSense data)
startTs = max(container['data']['vicon']['pose6q']['ts'][0], container['data']['rs']['pose6q']['ts'][0])
stopTs = min(container['data']['vicon']['pose6q']['ts'][-1], container['data']['rs']['pose6q']['ts'][-1])
numSteps = round(np.mean([len(container['data']['vicon']['pose6q']['ts']), len(container['data']['rs']['pose6q']['ts'])]))
timeWindow = np.linspace(startTs, stopTs, num=numSteps)

# Interpolate the data only in the common time window
slerperVicon = Slerp(times = container['data']['vicon']['pose6q']['ts'], rotations = Rotation.from_quat(container['data']['vicon']['pose6q']['rotation'][:, q2r_order]))
rotationsVicon = slerperVicon(timeWindow).as_quat()[:, r2q_order]

slerperRS = Slerp(times = container['data']['rs']['pose6q']['ts'], rotations = Rotation.from_quat(container['data']['rs']['pose6q']['rotation'][:, q2r_order]))
rotationsRS = slerperRS(timeWindow).as_quat()[:, r2q_order]

# Plot RealSense and Vicon orientation before and after the interpolation
import matplotlib.pyplot as plt

def plotQuaternionRSV(tsVicon, rotationsVicon_raw, rotationsVicon, tsRS, rotationsRS_raw, rotationsRS, datasetName, trialName):
    fig = plt.figure()
    fig.suptitle("Vicon and RealSense Orientation\n (" + datasetName + "/" + trialName + ")", fontsize=20, fontweight='bold')

    ax11 = plt.subplot(4, 2, 1)
    plt.plot(tsVicon, rotationsVicon_raw[:, 0], color='k', alpha=0.25)
    plt.plot(timeWindow, rotationsVicon[:, 0], color='k', linestyle='dotted', linewidth=2)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('w_Vicon', fontsize=10, fontweight='bold')
    plt.legend(['raw', 'interpolated'])
    plt.grid(True)

    plt.subplot(4, 2, 3, sharex=ax11)
    plt.plot(tsVicon, rotationsVicon_raw[:, 1], color='r', alpha=0.25)
    plt.plot(timeWindow, rotationsVicon[:, 1], color='r', linestyle='dotted', linewidth=2)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('x_Vicon', fontsize=10, fontweight='bold')
    plt.legend(['raw', 'interpolated'])
    plt.grid(True)

    plt.subplot(4, 2, 5, sharex=ax11)
    plt.plot(tsVicon, rotationsVicon_raw[:, 2], color='g', alpha=0.25)
    plt.plot(timeWindow, rotationsVicon[:, 2], color='g', linestyle='dotted', linewidth=2)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('y_Vicon', fontsize=10, fontweight='bold')
    plt.legend(['raw', 'interpolated'])
    plt.grid(True)

    plt.subplot(4, 2, 7, sharex=ax11)
    plt.plot(tsVicon, rotationsVicon_raw[:, 3], color='b', alpha=0.25)
    plt.plot(timeWindow, rotationsVicon[:, 3], color='b', linestyle='dotted', linewidth=2)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('z_Vicon', fontsize=10, fontweight='bold')
    plt.legend(['raw', 'interpolated'])
    plt.grid(True)

    ax11 = plt.subplot(4, 2, 2)
    plt.plot(tsRS, rotationsRS_raw[:, 0], color='k', alpha=0.25)
    plt.plot(timeWindow, rotationsRS[:, 0], color='k', linestyle='dotted', linewidth=2)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('w_RealSense', fontsize=10, fontweight='bold')
    plt.legend(['raw', 'interpolated'])
    plt.grid(True)

    plt.subplot(4, 2, 4, sharex=ax11)
    plt.plot(tsRS, rotationsRS_raw[:, 1], color='r', alpha=0.25)
    plt.plot(timeWindow, rotationsRS[:, 1], color='r', linestyle='dotted', linewidth=2)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('x_RealSense', fontsize=10, fontweight='bold')
    plt.legend(['raw', 'interpolated'])
    plt.grid(True)

    plt.subplot(4, 2, 6, sharex=ax11)
    plt.plot(tsRS, rotationsRS_raw[:, 2], color='g', alpha=0.25)
    plt.plot(timeWindow, rotationsRS[:, 2], color='g', linestyle='dotted', linewidth=2)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('y_RealSense', fontsize=10, fontweight='bold')
    plt.legend(['raw', 'interpolated'])
    plt.grid(True)

    plt.subplot(4, 2, 8, sharex=ax11)
    plt.plot(tsRS, rotationsRS_raw[:, 3], color='b', alpha=0.25)
    plt.plot(timeWindow, rotationsRS[:, 3], color='b', linestyle='dotted', linewidth=2)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('z_RealSense', fontsize=10, fontweight='bold')
    plt.legend(['raw', 'interpolated'])
    plt.grid(True)

    fig.subplots_adjust(top=0.9)
    fig.align_ylabels()

    plt.show()

tsVicon = container['data']['vicon']['pose6q']['ts']
rotationsVicon_raw = container['data']['vicon']['pose6q']['rotation']

tsRS = container['data']['rs']['pose6q']['ts']
rotationsRS_raw = container['data']['rs']['pose6q']['rotation']

%matplotlib auto
plotQuaternionRSV(tsVicon, rotationsVicon_raw, rotationsVicon, tsRS, rotationsRS_raw, rotationsRS, datasetName, trialName)

# %%
