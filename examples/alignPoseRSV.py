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

#%% Define function for plotting a set of points in a 3x1 grid
import matplotlib.pyplot as plt

def plotPointSet(tsA, pointsA, title, datasetName, trialName, filePath):
    fig = plt.figure(figsize=(13.33,7.5), dpi=96)
    fig.suptitle(title + "\n (" + datasetName + "/" + trialName + ")", fontsize=20, fontweight='bold')

    ax11 = plt.subplot(3, 1, 1)
    plt.plot(tsA, pointsA[:, 0], color='r', linestyle='dotted', linewidth=2)
    plt.ylabel('x [m]', fontsize=10, fontweight='bold')
    plt.grid(True)

    plt.subplot(3, 1, 2, sharex=ax11)
    plt.plot(tsA, pointsA[:, 1], color='g', linestyle='dotted', linewidth=2)
    plt.ylabel('y [m]', fontsize=10, fontweight='bold')
    plt.grid(True)

    plt.subplot(3, 1, 3, sharex=ax11)
    plt.plot(tsA, pointsA[:, 2], color='b', linestyle='dotted', linewidth=2)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('z [m]', fontsize=10, fontweight='bold')
    plt.grid(True)

    fig.subplots_adjust(top=0.9)
    fig.align_ylabels()

    plt.savefig(filePath, bbox_inches='tight')
    plt.show()

#%% Define function for plotting two sets of points in a 3x1 grid
import matplotlib.pyplot as plt

def plotPointSets3x1(tsA, tsB, pointsA, pointsB, legendA, legendB, title, datasetName, trialName, filePath):
    fig = plt.figure(figsize=(13.33, 7.5), dpi=96)
    fig.suptitle(title + "\n (" + datasetName + "/" + trialName + ")", fontsize=20, fontweight='bold')

    ax11 = plt.subplot(3, 1, 1)
    plt.plot(tsA, pointsA[:, 0], color='r', linestyle='dotted', linewidth=2)
    plt.plot(tsB, pointsB[:, 0], color='r', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.ylabel('x [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)

    plt.subplot(3, 1, 2, sharex=ax11)
    plt.plot(tsA, pointsA[:, 1], color='g', linestyle='dotted', linewidth=2)
    plt.plot(tsB, pointsB[:, 1], color='g', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.ylabel('y [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)

    plt.subplot(3, 1, 3, sharex=ax11)
    plt.plot(tsA, pointsA[:, 2], color='b', linestyle='dotted', linewidth=2)
    plt.plot(tsB, pointsB[:, 2], color='b', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('z [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)

    fig.subplots_adjust(top=0.9)
    fig.align_ylabels()

    plt.savefig(filePath, bbox_inches='tight')
    plt.show()

#%% Define function for plotting four sets of points in a 3x2 grid
import matplotlib.pyplot as plt

def plotPointSets3x2(tsA, tsB, tsAA, tsBB, pointsA, pointsB, pointsAA, pointsBB, legendA, legendB, legendAA, legendBB, title, datasetName, trialName, filePath):
    fig = plt.figure(figsize=(13.33, 7.5), dpi=96)
    fig.suptitle(title + "\n (" + datasetName + "/" + trialName + ")", fontsize=20, fontweight='bold')

    ax11 = plt.subplot(3, 2, 1)
    plt.plot(tsA, pointsA[:, 0], linestyle='dotted', linewidth=2, color='r')
    plt.plot(tsAA, pointsAA[:, 0], linestyle='dotted', linewidth=2, color='r', alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('x [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendAA])
    plt.grid(True)

    plt.subplot(3, 2, 3, sharex=ax11)
    plt.plot(tsA, pointsA[:, 1], linestyle='dotted', linewidth=2, color='g')
    plt.plot(tsAA, pointsAA[:, 1], linestyle='dotted', linewidth=2, color='g', alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('y [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendAA])
    plt.grid(True)

    plt.subplot(3, 2, 5, sharex=ax11)
    plt.plot(tsA, pointsA[:, 2], linestyle='dotted', linewidth=2, color='b')
    plt.plot(tsAA, pointsAA[:, 2], linestyle='dotted', linewidth=2, color='b', alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('z [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendAA])
    plt.grid(True)

    ax11 = plt.subplot(3, 2, 2)
    plt.plot(tsB, pointsB[:, 0], linestyle='dotted', linewidth=2, color='r')
    plt.plot(tsBB, pointsBB[:, 0], linestyle='dotted', linewidth=2, color='r', alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('x [m]', fontsize=10, fontweight='bold')
    plt.legend([legendB, legendBB])
    plt.grid(True)

    plt.subplot(3, 2, 4, sharex=ax11)
    plt.plot(tsB, pointsB[:, 1], linestyle='dotted', linewidth=2, color='g')
    plt.plot(tsBB, pointsBB[:, 1], linestyle='dotted', linewidth=2, color='g', alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('y [m]', fontsize=10, fontweight='bold')
    plt.legend([legendB, legendBB])
    plt.grid(True)

    plt.subplot(3, 2, 6, sharex=ax11)
    plt.plot(tsB, pointsB[:, 2], linestyle='dotted', linewidth=2, color='b')
    plt.plot(tsBB, pointsBB[:, 2], linestyle='dotted', linewidth=2, color='b', alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('z [m]', fontsize=10, fontweight='bold')
    plt.legend([legendB, legendBB])
    plt.grid(True)

    fig.subplots_adjust(top=0.9)
    fig.align_ylabels()

    plt.savefig(filePath, bbox_inches='tight')
    plt.show()

#%% Define function for plotting two sets of quaternions in a 3x1 grid
import matplotlib.pyplot as plt

def plotQuaternions(tsA, tsB, rotationsA, rotationsB, legendA, legendB, title, datasetName, trialName, filePath):
    fig = plt.figure(figsize=(13.33, 7.5), dpi=96)
    fig.suptitle(title + "\n (" + datasetName + "/" + trialName + ")", fontsize=20, fontweight='bold')

    ax11 = plt.subplot(4, 1, 1)
    plt.plot(tsA, rotationsA[:, 0], color='k', linestyle='dotted', linewidth=2)
    plt.plot(tsB, rotationsB[:, 0], color='k', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.ylabel('w', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)

    plt.subplot(4, 1, 2, sharex=ax11)
    plt.plot(tsA, rotationsA[:, 1], color='r', linestyle='dotted', linewidth=2)
    plt.plot(tsB, rotationsB[:, 1], color='r', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.ylabel('x', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)

    plt.subplot(4, 1, 3, sharex=ax11)
    plt.plot(tsA, rotationsA[:, 2], color='g', linestyle='dotted', linewidth=2)
    plt.plot(tsB, rotationsB[:, 2], color='g', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.ylabel('y', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)

    plt.subplot(4, 1, 4, sharex=ax11)
    plt.plot(tsA, rotationsA[:, 3], color='b', linestyle='dotted', linewidth=2)
    plt.plot(tsB, rotationsB[:, 3], color='b', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('z', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)

    fig.subplots_adjust(top=0.9)
    fig.align_ylabels()

    plt.savefig(filePath, bbox_inches='tight')
    plt.show()

#%% Define function for plotting two sets of points in 3D
from mpl_toolkits.mplot3d import Axes3D

def plotPointSets3D(pointsA, pointsB, legendA, legendB, title, datasetName, trialName, filePath):

    fig = plt.figure(figsize=(13.33, 7.5), dpi=96)
    fig.suptitle(title + "\n (" + datasetName + "/" + trialName + ")", fontsize=20, fontweight='bold')

    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pointsA[:,0], pointsA[:,1], pointsA[:,2], marker=".", color="#63ACBE")
    ax.scatter(pointsB[:,0], pointsB[:,1], pointsB[:,2], marker=".", color="#EE442F", alpha=0.5)

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')

    plt.legend([legendA, legendB])

    plt.savefig(filePath, bbox_inches='tight')
    plt.show()

#%% Define function for 3D rigid transform

# Reference: https://gist.github.com/oshea00/dfb7d657feca009bf4d095d4cb8ea4be
import numpy as np
from math import sqrt

# Implements Kabsch algorithm - best fit.
# Input:
#     Nominal  A Nx3 matrix of points
#     Measured B Nx3 matrix of points
# Returns R, t
# R = 3x3 rotation matrix (B to A)
# t = 3x1 translation vector (B to A)
def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0];  # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # center the points
    Ac = A - np.tile(centroid_A, (N, 1))
    Bc = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = np.transpose(Bc) * Ac

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = Vt.T * U.T

    t = -R * centroid_B.T + centroid_A.T

    return R, t

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
import os, sys

# Get os-specific path for local libraries (YMMV)
prefix = 'C:/' if os.name == 'nt' else '/home/adinale/'

# Add path to bimvee library
sys.path.append(os.path.join(prefix, 'src/bimvee/'))
# Add path to mustard
sys.path.append(os.path.join(prefix, 'src/mustard'))

#%% Load StEFI and Vicon data
from bimvee.importIitYarp import importIitYarp
from bimvee.importIitVicon import separateMarkersFromSegments
from bimvee.split import splitByLabel

datasetPath = "Documents/Event-Driven_Dataset"
datasetName = "chessboardViconRealSense"
trialName = "2020-06-26_trial01"

filePathOrName = os.path.join(prefix, datasetPath + "/" + datasetName + "/" + trialName)
containerYarp = importIitYarp(filePathOrName=filePathOrName, tsBits=30, zeroTimestamps=True)

# Separate markers (point3) from segments (pose6q)
containerYarp[1]['data']['vicon']['pose6q'] = separateMarkersFromSegments(containerYarp[1]['data']['vicon']['pose6q'])['pose6q']

#%%
# Remove null poses
selected = np.any(containerYarp[1]['data']['vicon']['pose6q']['point'] == np.zeros((1, 3)), axis = 1)
for key in containerYarp[1]['data']['vicon']['pose6q'].keys():
    if key != 'tsOffset':
        containerYarp[1]['data']['vicon']['pose6q'][key] = containerYarp[1]['data']['vicon']['pose6q'][key][~selected]

# Select only poses representing STEFI
containerYarp[1]['data']['vicon']['pose6q'] =  splitByLabel(containerYarp[1]['data']['vicon']['pose6q'], 'bodyId')['Subj_StEFI::Seg_StEFI']

# Remove repeated poses
_, selected = np.unique(containerYarp[1]['data']['vicon']['pose6q']['ts'], return_index=True)
for key in containerYarp[1]['data']['vicon']['pose6q'].keys():
    if key != 'tsOffset':
        containerYarp[1]['data']['vicon']['pose6q'][key] = containerYarp[1]['data']['vicon']['pose6q'][key][selected]

# Clean-up
del selected, key

#%% Load RealSense data (ONLY poses)
from bimvee.importRpgDvsRos import importRpgDvsRos

trialNameRS = "2020-06-26_trial01/RealSense/trial01.bag"
bagname = os.path.join(prefix, datasetPath + "/" + datasetName + "/" + trialNameRS)
template = {'rs': {'pose6q': '/device_0/sensor_0/Pose_0/pose/transform/data'}}

containerRealsense = importRpgDvsRos(filePathOrName=bagname, zeroTimestamps=True, template=template)

#%% Plot Vicon and RealSense RAW data
import copy

containerRaw = {'info': {'vicon': copy.deepcopy(containerYarp[1]['info']),
                         'rs': copy.deepcopy(containerRealsense['info'])},
                'data': {'vicon': copy.deepcopy(containerYarp[1]['data']['vicon']),
                         'rs': copy.deepcopy(containerRealsense['data']['rs'])}}

pointsRSraw_MOD = np.array([-containerRaw['data']['rs']['pose6q']['point'][:,0] + containerRaw['data']['vicon']['pose6q']['point'][0,0],
                             containerRaw['data']['rs']['pose6q']['point'][:,1],
                             containerRaw['data']['rs']['pose6q']['point'][:,2]]).T

%matplotlib auto
plotPointSets3x1(containerRaw['data']['vicon']['pose6q']['ts'],
                 containerRaw['data']['rs']['pose6q']['ts'],
                 containerRaw['data']['vicon']['pose6q']['point'],
                 pointsRSraw_MOD,
                 "Vicon", "RealSense", "Vicon and RealSense points after importing (modified X)", datasetName, trialName, os.path.join(filePathOrName, '01_RSV_rawData_MOD.png'))

plotPointSets3x1(containerRaw['data']['vicon']['pose6q']['ts'],
                 containerRaw['data']['rs']['pose6q']['ts'],
                 containerRaw['data']['vicon']['pose6q']['point'],
                 containerRaw['data']['rs']['pose6q']['point'],
                 "Vicon", "RealSense", "Vicon and RealSense points after importing", datasetName, trialName, os.path.join(filePathOrName, '02_RSV_rawData.png'))

plotPointSets3D(containerRaw['data']['vicon']['pose6q']['point'],
                containerRaw['data']['rs']['pose6q']['point'],
                "Vicon", "RealSense", "Vicon and RealSense points after importing", datasetName, trialName, os.path.join(filePathOrName, '03_RSV_rawData3D.png'))

#%% Align RealSense data w.r.t. StEFI and Vicon data in TIME
import copy
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

        containerRealsense['info']['tsOffsetFromInfo'] = frames.get_timestamp()/1000 # milliseconds to seconds
finally:
    pipeline.stop()

# Compute and apply the temporal offset to the RealSense data
tsOffsetFromInfo = min(containerYarp[0]['info']['tsOffsetFromInfo'], containerYarp[1]['info']['tsOffsetFromInfo'])
tsOffset = containerRealsense['info']['tsOffsetFromInfo'] - tsOffsetFromInfo

containerRealsense['data']['rs']['pose6q']['ts'] = containerRealsense['data']['rs']['pose6q']['ts'] + tsOffset
containerRealsense['data']['rs']['pose6q']['tsOffset'] = tsOffset

#%% Plot Vicon and RealSense data after time alignment
import copy
import numpy as np
from bimvee.importIitVicon import separateMarkersFromSegments
from bimvee.split import splitByLabel

container = {'info': {'vicon': copy.deepcopy(containerYarp[1]['info']),
                      'rs': copy.deepcopy(containerRealsense['info'])},
             'data': {'vicon': copy.deepcopy(containerYarp[1]['data']['vicon']),
                      'rs': copy.deepcopy(containerRealsense['data']['rs'])}}

pointsRS_MOD = np.array([-container['data']['rs']['pose6q']['point'][:,0] + container['data']['vicon']['pose6q']['point'][0,0],
                          container['data']['rs']['pose6q']['point'][:,1],
                          container['data']['rs']['pose6q']['point'][:,2]]).T

plotPointSets3x1(container['data']['vicon']['pose6q']['ts'],
                 container['data']['rs']['pose6q']['ts'],
                 container['data']['vicon']['pose6q']['point'],
                 pointsRS_MOD,
                 "Vicon", "RealSense", "Vicon and RealSense points after time alignment (container - modified X)", datasetName, trialName, os.path.join(filePathOrName, '04_RSV_rawDataAlignedTime_MOD.png'))

plotPointSets3x1(container['data']['vicon']['pose6q']['ts'],
                 container['data']['rs']['pose6q']['ts'],
                 container['data']['vicon']['pose6q']['point'],
                 container['data']['rs']['pose6q']['point'],
                 "Vicon", "RealSense", "Vicon and RealSense points after time alignment (container)", datasetName, trialName, os.path.join(filePathOrName, '05_RSV_rawDataAlignedTime.png'))

plotPointSets3D(container['data']['vicon']['pose6q']['point'],
                container['data']['rs']['pose6q']['point'],
                "Vicon", "RealSense", "Vicon and RealSense points after time alignment (container)", datasetName, trialName, os.path.join(filePathOrName, '06_RSV_rawData3DAlignedTime.png'))

#%% Find a common time window between Vicon and RealSense data
from bimvee.split import cropTime

# Find the first common time and the last common time
startTs = max(container['data']['vicon']['pose6q']['ts'][0], container['data']['rs']['pose6q']['ts'][0])
stopTs = min(container['data']['vicon']['pose6q']['ts'][-1], container['data']['rs']['pose6q']['ts'][-1])
#TODO: check who actually comes first! trailing at beginning

# Crop data to have ROUGHLY the same common time window
container['data']['vicon']['pose6q'] = cropTime(container['data']['vicon']['pose6q'], startTime=startTs, stopTime=stopTs, zeroTime=False)
container['data']['rs']['pose6q'] = cropTime(container['data']['rs']['pose6q'], startTime=startTs, stopTime=stopTs, zeroTime=False)

# Guarantee a feasible time window for the interpolation (i.e. it must be within the limits of both Vicon and RealSense data)
tsVicon = container['data']['vicon']['pose6q']['ts']
tsRS = container['data']['rs']['pose6q']['ts']
startTs = max(tsVicon[0], tsRS[0])
stopTs = min(tsVicon[-1], tsRS[-1])
numSteps = round(np.mean([len(tsVicon), len(tsRS)]))
timeWindow = np.linspace(startTs, stopTs, num=numSteps)

#%% Interpolate with a Cubic Spline the Vicon and RealSense points
from scipy.interpolate import CubicSpline

pointsVicon_raw = copy.deepcopy(container['data']['vicon']['pose6q']['point'])
pointsRS_raw = copy.deepcopy(container['data']['rs']['pose6q']['point'])

cx = CubicSpline(x = tsVicon, y = pointsVicon_raw[:,0])
cy = CubicSpline(x = tsVicon, y = pointsVicon_raw[:,1])
cz = CubicSpline(x = tsVicon, y = pointsVicon_raw[:,2])
pointsVicon = np.transpose([cx(timeWindow), cy(timeWindow), cz(timeWindow)])

cx = CubicSpline(x = tsRS, y = pointsRS_raw[:,0])
cy = CubicSpline(x = tsRS, y = pointsRS_raw[:,1])
cz = CubicSpline(x = tsRS, y = pointsRS_raw[:,2])
pointsRS = np.transpose([cx(timeWindow), cy(timeWindow), cz(timeWindow)])

del cx, cy, cz

pointsRS_MOD2 = np.array([-pointsRS[:,0] + pointsVicon[0,0],
                           pointsRS[:,1],
                           pointsRS[:,2]]).T

# Plot Vicon and RealSense data within the container
plotPointSets3x1(timeWindow,
                 timeWindow,
                 pointsVicon,
                 pointsRS_MOD2,
                 "Vicon", "RealSense", "Vicon and RealSense points after Cubic interpolation (modified X)", datasetName, trialName, os.path.join(filePathOrName, '07_RSV_interpolatedData_MOD.png'))

# Plot Vicon and RealSense points after Cubic interpolation
plotPointSets3x1(timeWindow,
                 timeWindow,
                 pointsVicon,
                 pointsRS,
                 "Vicon", "RealSense", "Vicon and RealSense points after Cubic interpolation", datasetName, trialName, os.path.join(filePathOrName, '08_RSV_interpolatedData.png'))

# Plot Vicon and RealSense data in 3D
plotPointSets3D(pointsVicon,
                pointsRS,
                "Vicon", "RealSense", "Vicon and RealSense points after Cubic interpolation", datasetName, trialName, os.path.join(filePathOrName, '09_RSV_interpolatedData3D.png'))

#%% Interpolate with Slerp the Vicon and RealSense rotations
# -------------------------------------
# | IGNORE THE FOLLOWING CODE FOR NOW |
# -------------------------------------

# from scipy.spatial.transform import Sler
# from scipy.spatial.transform import Rotation

# rotationsVicon_raw = copy.deepcopy(container['data']['vicon']['pose6q']['rotation'])
# rotationsRS_raw = copy.deepcopy(container['data']['rs']['pose6q']['rotation'])

# # Define quaternion format
# q2r_order = [1, 2, 3, 0] # xyzw
# r2q_order = [3, 0, 1, 2] # wxyz

# # Interpolate the data only in the common time window
# slerperVicon = Slerp(times = tsVicon, rotations = Rotation.from_quat(rotationsVicon_raw[:, q2r_order]))
# rotationsVicon_quat = slerperVicon(timeWindow).as_quat()[:, r2q_order]
# rotationsVicon_matrix = slerperVicon(timeWindow).as_matrix()

# slerperRS = Slerp(times = tsRS, rotations = Rotation.from_quat(rotationsRS_raw[:, q2r_order]))
# rotationsRS_quat = slerperRS(timeWindow).as_quat()[:, r2q_order]
# rotationsRS_matrix = slerperRS(timeWindow).as_matrix()

# # Plot Vicon and RealSense orientations after Slerper innterpolation
# # %matplotlib auto
# plotQuaternions(timeWindow, rotationsVicon_quat, rotationsRS_quat, 'vicon', 'realsense', 'Vicon and RealSense orientations after SLERP', datasetName, trialName)

#%% Compute transformation matrix between Vicon and RealSense data

A = copy.deepcopy(np.asmatrix(pointsVicon))
B = copy.deepcopy(np.asmatrix(pointsRS))

# Recover R and t
ret_R, ret_t = rigid_transform_3D(A, B)

# Compare the recovered R and t with the original
n = len(pointsRS)
B2 = (ret_R * B.T) + np.tile(ret_t, (1, n))

# Find the root mean squared error
B2 = (ret_R * B.T) + np.tile(ret_t, (1, n))
B2 = B2.T
err = A - B2
err = np.multiply(err, err)
err = np.sum(err)
rmse = sqrt(err / n);

plotPointSets3x1(timeWindow, timeWindow, A, B2, "Vicon", "RealSense", "Vicon and RealSense points after rigid transformation", datasetName, trialName, os.path.join(filePathOrName, '10_RSV_transformedData.png'))

plotPointSets3D(A, B2, "Vicon", "RealSense", "Vicon and RealSense points after rigid transformation", datasetName, trialName, os.path.join(filePathOrName, '11_RSV_transformedData3D.png'))

err_x = A[:, 0] - B2[:, 0]
err_y = A[:, 1] - B2[:, 1]
err_z = A[:, 2] - B2[:, 2]
err = np.concatenate((err_x, err_y, err_z), axis=1)

plotPointSet(timeWindow, err, "Rigid Transformation Error", datasetName, trialName, os.path.join(filePathOrName, '12_RSV_transformationError.png'))

#%%
