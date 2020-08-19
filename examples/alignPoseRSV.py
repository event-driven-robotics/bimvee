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

def plotPointSet(tsA, pointsA, title, datasetName, trialName):
    fig = plt.figure()
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

    plt.show()

#%% Define function for plotting two sets of points in a 3x1 grid
import matplotlib.pyplot as plt

def plotPointSets3x1(tsA, tsB, pointsA, pointsB, legendA, legendB, title, datasetName, trialName):
    fig = plt.figure()
    fig.suptitle(title + "\n (" + datasetName + "/" + trialName + ")", fontsize=20, fontweight='bold')

    ax11 = plt.subplot(3, 1, 1)
    plt.plot(tsA, pointsA[:, 0], color='r', linestyle='dotted', linewidth=2)
    plt.plot(tsB, pointsB[:, 0], color='r', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.ylabel('x [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)
    # plt.ylim(-1.5, 0.4)

    plt.subplot(3, 1, 2, sharex=ax11)
    plt.plot(tsA, pointsA[:, 1], color='g', linestyle='dotted', linewidth=2)
    plt.plot(tsB, pointsB[:, 1], color='g', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.ylabel('y [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)
    # plt.ylim(-0.5, 1.5)

    plt.subplot(3, 1, 3, sharex=ax11)
    plt.plot(tsA, pointsA[:, 2], color='b', linestyle='dotted', linewidth=2)
    plt.plot(tsB, pointsB[:, 2], color='b', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('z [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)
    # plt.ylim(-0.3, 1.3)

    fig.subplots_adjust(top=0.9)
    fig.align_ylabels()

    plt.show()

#%% Define function for plotting four sets of points in a 3x2 grid
import matplotlib.pyplot as plt

def plotPointSets3x2(tsA, tsB, tsAA, tsBB, pointsA, pointsB, pointsAA, pointsBB, legendA, legendB, legendAA, legendBB, title, datasetName, trialName):
    fig = plt.figure()
    fig.suptitle(title + "\n (" + datasetName + "/" + trialName + ")", fontsize=20, fontweight='bold')

    ax11 = plt.subplot(3, 2, 1)
    plt.plot(tsA, pointsA[:, 0], linestyle='dotted', linewidth=2, color='r')
    plt.plot(tsAA, pointsAA[:, 0], linestyle='dotted', linewidth=2, color='r', alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('x [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendAA])
    plt.grid(True)

    plt.subplot(3, 2, 3, sharex=ax11)
    plt.plot(tsA, pointsA[:, 1], linestyle='dotted', linewidth=2, color='r')
    plt.plot(tsAA, pointsAA[:, 1], linestyle='dotted', linewidth=2, color='r', alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('y [m]', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendAA])
    plt.grid(True)

    plt.subplot(3, 2, 5, sharex=ax11)
    plt.plot(tsA, pointsA[:, 2], linestyle='dotted', linewidth=2, color='r')
    plt.plot(tsAA, pointsAA[:, 2], linestyle='dotted', linewidth=2, color='r', alpha=0.25)
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
    plt.plot(tsB, pointsB[:, 1], linestyle='dotted', linewidth=2, color='r')
    plt.plot(tsBB, pointsBB[:, 1], linestyle='dotted', linewidth=2, color='r', alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('y [m]', fontsize=10, fontweight='bold')
    plt.legend([legendB, legendBB])
    plt.grid(True)

    plt.subplot(3, 2, 6, sharex=ax11)
    plt.plot(tsB, pointsB[:, 2], linestyle='dotted', linewidth=2, color='r')
    plt.plot(tsBB, pointsBB[:, 2], linestyle='dotted', linewidth=2, color='r', alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('z [m]', fontsize=10, fontweight='bold')
    plt.legend([legendB, legendBB])
    plt.grid(True)

    fig.subplots_adjust(top=0.9)
    fig.align_ylabels()

    plt.show()

#%% Define function for plotting two sets of quaternions in a 3x1 grid
import matplotlib.pyplot as plt

def plotQuaternions(tsA, tsB, rotationsA, rotationsB, legendA, legendB, title, datasetName, trialName):
    fig = plt.figure()
    fig.suptitle(title + "\n (" + datasetName + "/" + trialName + ")", fontsize=20, fontweight='bold')

    ax11 = plt.subplot(4, 1, 1)
    plt.plot(tsA, rotationsA[:, 0], color='k', linestyle='dotted', linewidth=2)
    plt.plot(tsB, rotationsB[:, 0], color='k', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.ylabel('w', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)
    # plt.ylim(-1, 1.3)

    plt.subplot(4, 1, 2, sharex=ax11)
    plt.plot(tsA, rotationsA[:, 1], color='r', linestyle='dotted', linewidth=2)
    plt.plot(tsB, rotationsB[:, 1], color='r', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.ylabel('x', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)
    # plt.ylim(-1, 1)

    plt.subplot(4, 1, 3, sharex=ax11)
    plt.plot(tsA, rotationsA[:, 2], color='g', linestyle='dotted', linewidth=2)
    plt.plot(tsB, rotationsB[:, 2], color='g', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.ylabel('y', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)
    # plt.ylim(-1, 1)

    plt.subplot(4, 1, 4, sharex=ax11)
    plt.plot(tsA, rotationsA[:, 3], color='b', linestyle='dotted', linewidth=2)
    plt.plot(tsB, rotationsB[:, 3], color='b', linestyle='dotted', linewidth=2, alpha=0.25)
    plt.xlabel('Time [s]', fontsize=10, fontweight='bold')
    plt.ylabel('z', fontsize=10, fontweight='bold')
    plt.legend([legendA, legendB])
    plt.grid(True)
    # plt.ylim(-s1, 1)

    fig.subplots_adjust(top=0.9)
    fig.align_ylabels()

    plt.show()

#%% Define function for plotting two sets of points in 3D
from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.cm as cm

def plotPointSets3D(pointsA, pointsB, legendA, legendB, title, datasetName, trialName):

    # cA = range(0, len(pointsA[:,0]))
    # cB = range(0, len(pointsB[:,0]))

    fig = plt.figure()
    fig.suptitle(title + "\n (" + datasetName + "/" + trialName + ")", fontsize=20, fontweight='bold')

    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pointsA[:,0], pointsA[:,1], pointsA[:,2], marker=".", color="#63ACBE")#c=cA, cmap=cm.YlOrRd_r)
    ax.scatter(pointsB[:,0], pointsB[:,1], pointsB[:,2], marker=".", color="#EE442F", alpha=0.5)#c=cB ,cmap=cm.Blues_r)

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')

    plt.legend([legendA, legendB])

    plt.show()

#%% Define function for 3D rigid transform
import numpy as np
from math import sqrt

# Implements Kabsch algorithm - best fit.
# Input:
#     Nominal  A Nx3 matrix of points
#     Measured B Nx3 matrix of points
# Returns Ac, Bc, R, t
# Ac = Nx3 matrix of points obtained by subtracting the centroid from A
# Bc = Nx3 matrix of points obtained by subtracting the centroid from B
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

    return Ac, Bc, R, t

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
containerYarp = importIitYarp(filePathOrName=filePathOrName, tsBits=30)

# Separate markers (pose3) from segments (pose6q)
containerYarp[1]['data']['vicon']['pose6q'] = separateMarkersFromSegments(containerYarp[1]['data']['vicon']['pose6q'])['pose6q']

# Remove null poses
selected = np.any(containerYarp[1]['data']['vicon']['pose6q']['point'] == np.zeros((1, 3)), axis = 1)
for key in containerYarp[1]['data']['vicon']['pose6q'].keys():
    containerYarp[1]['data']['vicon']['pose6q'][key] = containerYarp[1]['data']['vicon']['pose6q'][key][~selected]

# Select only poses representing STEFI
containerYarp[1]['data']['vicon']['pose6q'] =  splitByLabel(containerYarp[1]['data']['vicon']['pose6q'], 'bodyId')['Subj_StEFI::Seg_StEFI']

# Remove repeated poses
_, selected = np.unique(containerYarp[1]['data']['vicon']['pose6q']['ts'], return_index=True)
for key in containerYarp[1]['data']['vicon']['pose6q'].keys():
    containerYarp[1]['data']['vicon']['pose6q'][key] = containerYarp[1]['data']['vicon']['pose6q'][key][selected]

# Clean-up
del selected, key

# %matplotlib auto
# plotPointSet(containerYarp[1]['data']['vicon']['pose6q']['ts'], containerYarp[1]['data']['vicon']['pose6q']['point'], "Vicon Data after running importIitYarp", datasetName, trialName)

#%% Load RealSense data (ONLY poses)
from bimvee.importRpgDvsRos import importRpgDvsRos

trialNameRS = "2020-06-26_trial01/RealSense/trial01.bag"
bagname = os.path.join(prefix, datasetPath + "/" + datasetName + "/" + trialNameRS)
containerRealsense = importRpgDvsRos(filePathOrName=bagname, zeroTimestamps=True)

# plotPointSet(containerRealsense['data']['/device_0/sensor_0/Pose_0/pose/transform/data']['pose6q']['ts'], containerRealsense['data']['/device_0/sensor_0/Pose_0/pose/transform/data']['pose6q']['point'], "RealSense Data after running importRpgDvsRos", datasetName, trialName)

%matplotlib auto
plotPointSets3x1(containerYarp[1]['data']['vicon']['pose6q']['ts'],
                 containerRealsense['data']['/device_0/sensor_0/Pose_0/pose/transform/data']['pose6q']['ts'],
                 containerYarp[1]['data']['vicon']['pose6q']['point'],
                 containerRealsense['data']['/device_0/sensor_0/Pose_0/pose/transform/data']['pose6q']['point'],
                 "Vicon", "RealSense", "Vicon and RealSense points after importing", datasetName, trialName)

# Plot Vicon and RealSense data in 3D
plotPointSets3D(containerYarp[1]['data']['vicon']['pose6q']['point'],
                containerRealsense['data']['/device_0/sensor_0/Pose_0/pose/transform/data']['pose6q']['point'],
                "Vicon", "RealSense", "Vicon and RealSense points after importing", datasetName, trialName)

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

        containerRealsense['info']['tsOffsetFromInfo'] = frames.get_timestamp()/1000 # milliseconds to seconds
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
# from bimvee.plot import plot

container = {'info': copy.deepcopy(containerYarp[1]['info']),
             'data': {'vicon': copy.deepcopy(containerYarp[1]['data']['vicon']),
                      'rs': copy.deepcopy(containerRealsense['data']['/device_0/sensor_0/Pose_0/pose/transform/data'])}}

# Plot the obtained container
# %matplotlib auto
# plot(container)

# Plot Vicon and RealSense data within the container
# plotPointSet(container['data']['vicon']['pose6q']['ts'], container['data']['vicon']['pose6q']['point'], "Vicon Data (container)", datasetName, trialName)
# plotPointSet(container['data']['rs']['pose6q']['ts'], container['data']['rs']['pose6q']['point'], "RealSense Data (container)", datasetName, trialName)
plotPointSets3x1(container['data']['vicon']['pose6q']['ts'],
                 container['data']['rs']['pose6q']['ts'],
                 container['data']['vicon']['pose6q']['point'],
                 container['data']['rs']['pose6q']['point'],
                 "Vicon", "RealSense", "Vicon and RealSense points after time alignment (container)", datasetName, trialName)

# Plot Vicon and RealSense data in 3D
plotPointSets3D(container['data']['vicon']['pose6q']['point'],
                container['data']['rs']['pose6q']['point'],
                "Vicon", "RealSense", "Vicon and RealSense points after time alignment (container)", datasetName, trialName)

#%% Find a common time windows between Vicon and RealSense data00
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

# Plot Vicon and RealSense points after Cubic interpolation
plotPointSets3x1(timeWindow,
                 timeWindow,
                 pointsVicon,
                 pointsRS,
                 "Vicon", "RealSense", "Vicon and RealSense points after Cubic interpolation", datasetName, trialName)

# Plot Vicon and RealSense data in 3D
plotPointSets3D(pointsVicon,
                pointsRS,
                "Vicon", "RealSense", "Vicon and RealSense points after Cubic interpolation", datasetName, trialName)

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
Ac, Bc, ret_R, ret_t = rigid_transform_3D(A, B)
plotPointSets3D(Ac, Bc, "Vicon", "RealSense", "Vicon and RealSense points after substracting centroid", datasetName, trialName)

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

plotPointSets3x1(timeWindow, timeWindow, A, B2, "Vicon", "RealSense", "Vicon and RealSense points after rigid transformation", datasetName, trialName)

plotPointSets3D(A, B2, "Vicon", "RealSense", "Vicon and RealSense points after rigid transformation", datasetName, trialName)

err_x = A[:, 0] - B2[:, 0]
err_y = A[:, 1] - B2[:, 1]
err_z = A[:, 2] - B2[:, 2]
err = np.concatenate((err_x, err_y, err_z), axis=1)

plotPointSet(timeWindow, err, "Rigid Transformation Error", datasetName, trialName)

###############
# STOP HERE ! #
###############

#%% Global Registration for rough point cloud alignment [http://www.open3d.org/docs/0.6.0/tutorial/Advanced/global_registration.html#ransac]
import open3d as o3d

# Create a PointCloud for Vicon points
source = o3d.geometry.PointCloud()
source.points = o3d.utility.Vector3dVector(pointsVicon)

# Create a PointCloud for RealSense points
target = o3d.geometry.PointCloud()
target.points = o3d.utility.Vector3dVector(pointsRS)

# trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                        #  [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0]) # yellowish
    target_temp.paint_uniform_color([0, 0.651, 0.929]) # blueish
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

# draw_registration_result(source, target, trans_init)
# del trans_init

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size, source, target):
    # print(":: Load two point clouds and disturb initial pose.")
    # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                          [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    # source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

voxel_size = 0.05 # means 5cm for this dataset
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, source, target)

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result

result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
print(result_ransac)
draw_registration_result(source_down, target_down, result_ransac.transformation)
draw_registration_result(source, target, result_ransac.transformation)

#%% Iterative Closest Point (ICP) for fine point cloud alignment [http://www.open3d.org/docs/release/tutorial/Basic/icp_registration.html?highlight=registration]

threshold = 0.001
trans_init = result_ransac.transformation

print("Initial alignment")
evaluation = o3d.registration.evaluate_registration(source, target, threshold, trans_init)
print(evaluation)

print("Apply point-to-point ICP")
reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration = 5000))

print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
draw_registration_result(source, target, reg_p2p.transformation)
