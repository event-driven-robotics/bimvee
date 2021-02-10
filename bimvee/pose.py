# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Sim Bamford
         Aiko Dinale

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
Functions for manipulating 3D 6DOF poses

A set of functions for manipulating dictionaries containing timestamped sets
of 3D poses.
The typical dictionary format is:

{
*	'ts': numpy array (n) of np.float64 - timestamps in seconds, assumed to be
                            sorted ascending,
*	'point': numpy array (n, 3) of np.float64, where each row contains [x, y, z],
*	'rotation': see below for different pose formats,
*   optionally, other fields, where each is an iterable with 1st dimension (n, ...)
}

So the nth point and the nth rotation represent the pose at the nth timestamp.

The rotation may be either:

* Unit quaternion: (n, 4) where each row contains [w, x, y, z] (NOTE! ordering
                          of quaternion elements may differ in other software
                          frameworks, e.g. ROS uses [x, y, z, w]
* Rotation matrix: (n, 3, 3)
* Rodrigues vector: (n, 3)
* Axis-Angle: (n, 1) this is the angle of axis-angle representation, there is
                     then a further field called 'axis' (n, 3)

We will name these formats differently depending on the rotation representation,
respectively:

pose6q
pose6mat
pose6rvec
pose6axang

In the case of 'mat', there is a further field 'transformation', (n, 4, 4)
which contains transformation matrices for each timestamp. In fact, the 'point'
and 'rotation' fields in this case are simply views to the appropriate parts
of 'transformation', i.e. [n, :3, 3] and [n, :3, :3] respectively.

There are functions to translate one rotation representation to another:

rotToMat(dict)
rotToVec(dict)
rotToQuat(dict)
rotToAxisAngle(dict)

Each of the above deduces the type from the dimensions of 'rotation' before
applying the appropriate transformation.

There are these additional functions, which apply to all samples in a pose dict:

interpolatePoses(dict, times=None, period=None, maxPeriod=None)
rotate(dict, rotation)
translate(dict, translation)
transform(dict, translation=None, rotation=None)
angleBetween(dict, rotation)
averageRotation(dict, weights=None)

There are also these forms, which work sample-wise between two dicts:

transform(dict1, rotation=dict2) - applies as if the dicts contained mats
                                    and the result were: dict2 @ dict1
angleBetween(dict1, rotation=dict2)

In both of the above cases, if the dicts do not have identical timstamps,
then the resulting dict has a result at each time in each dict
which is contained within the bounds of the timestamps in the other dict
- i.e. interpolation is used to align the data intime.

There are lower level functions which don't operate on dicts but rather
on the appropriate arrays and on singular samples; these are exposed and can
be imported for convenience.
"""

import numpy as np

# local imports
from .timestamps import sortDataTypeDictByTime
from .split import selectByBool

#%% Helper functions

def findRotationRepresentation(rotation):
    if rotation.shape[1] == 1:
        return 'axisAngle'
    elif rotation.shape[1] == 4:
        return 'quaternion'
    elif rotation.ndim == 2:
        return 'rVec'
    else:
        return 'mat'

def findRotationRepresentationSingle(rotation):
    if rotation.shape[0] == 4:
        return 'quaternion'
    elif rotation.ndim == 2:
        return 'mat'
    else:
        return 'rVec'

#%% Functions we want

def rotToRepresentation(inDict, rotationRepresentation):
    if rotationRepresentation == 'quaternion':
        return rotToQuat(inDict)
    elif rotationRepresentation == 'axisAngle':
        return rotToAxisAngle(inDict)
    elif rotationRepresentation == 'rVec':
        return rotToVec(inDict)
    else: #rotationRepresentation == 'mat'
        return rotToMat(inDict)


def rotToMat(inDict):
    rotation = inDict['rotation']
    rotationRepresentation = findRotationRepresentation(rotation)
    if rotationRepresentation == 'mat':
        # already a matrix
        return inDict

    mat = np.zeros((rotation.shape[0], 4, 4), dtype=np.float64)
    mat[:, 3, 3] = 1
    mat[:, :3, 3] = inDict['point']
    outDict = inDict.copy()
    outDict['point'] = mat[:, :3, 3]
    outDict['rotation'] = mat[:, :3, :3]
    outDict['transformation'] = mat

    if rotationRepresentation == 'axisAngle':
        angle = rotation
        axis = inDict['axis']
        x = axis[:, 0]
        y = axis[:, 1]
        z = axis[:, 2]
        sin = np.sin(angle)
        cos = np.cos(angle)
        outDict = inDict.copy()
        # following: https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
        mat[:, 0, 0] = cos + x**2 * (1 - cos)
        mat[:, 0, 1] = x * y * (1 - cos) - z * sin
        mat[:, 0, 2] = x * z * (1 - cos) + y * sin
        mat[:, 1, 0] = y * x * (1 - cos) + z * sin
        mat[:, 1, 1] = cos + y**2 * (1 - cos)
        mat[:, 1, 2] = y * z * (1 - cos) - x * sin
        mat[:, 2, 0] = z * x * (1 - cos) - y * sin
        mat[:, 2, 1] = z * y * (1 - cos) + x * sin
        mat[:, 2, 2] = cos + z**2 * (1 - cos)
    elif rotationRepresentation == 'quaternion':
        w = rotation[:, 0]
        x = rotation[:, 1]
        y = rotation[:, 2]
        z = rotation[:, 3]
        mat[:, 0, 0] = 1 - 2 * y**2 - 2 * z**2
        mat[:, 0, 1] = 2 * x * y - 2 * z * w
        mat[:, 0, 2] = 2 * x * z + 2 * y * w
        mat[:, 1, 0] = 2 * x * y + 2 * z * w
        mat[:, 1, 1] = 1 - 2 * x**2 - 2 * z**2
        mat[:, 1, 2] = 2 * y * z - 2 * x*w
        mat[:, 2, 0] = 2 * x * z - 2 * y*w
        mat[:, 2, 1] = 2 * y * z + 2 * x*w
        mat[:, 2, 2] = 1 - 2 * x**2 - 2 * y**2
    else: #rotationRepresentation == 'rVec'
        # following: https://github.com/robEllenberg/comps-plugins/blob/master/python/rodrigues.py
        # Start with identity matrix
        '''
        mat[:, 0, 0] = 1
        mat[:, 1, 1] = 1
        mat[:, 1, 2] = 1
        def S(n, mat):
            rotation
            Sn = np.zeros_like(mat)
            Sn[:, 0, 1] = -n[:, 2]
            Sn[:, 0, 2] = n[:, 1]
            Sn[:, 1, 0] = n[:, 2]
            Sn[:, 0, 2] = -n[:, 0]
            Sn[:, 2, 0] = -n[:, 1]
            Sn[:, 2, 1] = n[:, 0]
            return Sn

        theta = np.linalg.norm(rotation, axis=1)
        n = rotation / theta
        Sn = S(n, mat)
        Sr = S(r, mat)
        theta2 = theta**2
        cond = theta > 1e-30

        def dotAlongZerothAxis(S, mat): # TODO: there is probably a neat numpy way of doing this: tensordot? apply_along_axis?
            out = np.zeros_like(mat)
            for idx, slicee in enumerate(S):
                out[idx, :, :] = np.dot(S, S)
            return out

        mat = mat \
            + cond * ( np.sin(theta)*Sn + (1-np.cos(theta))*dotAlongZerothAxis(Sn, mat) ) \
            + (1-cond) * ((1-theta2/6)*Sr + (0.5-theta2/24)*dotAlongZerothAxis(Sr, mat) )
        '''
        pass
    return outDict

def rotToVec(inDict):
    rotation = inDict['rotation']
    rotationRepresentation = findRotationRepresentation(rotation)
    if rotationRepresentation == 'rVec':
        return inDict
    outDict = inDict.copy()
    if rotationRepresentation == 'axisAngle':
        raise NotImplementedError("Method not implemented")
    elif rotationRepresentation == 'quaternion':
        raise NotImplementedError("Method not implemented")
    elif rotationRepresentation == 'rVec':
        raise NotImplementedError("Method not implemented")
    else: #rotationRepresentation == 'mat'
        raise NotImplementedError("Method not implemented")
    return outDict

def rotToQuat(inDict):
    rotation = inDict['rotation']
    rotationRepresentation = findRotationRepresentation(rotation)
    if rotationRepresentation == 'quaternion':
        return inDict
    if rotationRepresentation == 'axisAngle':
        raise NotImplementedError("Method not implemented")
    elif rotationRepresentation == 'rVec':
        raise NotImplementedError("Method not implemented")
    else: #rotationRepresentation == 'mat'
        outDict = inDict.copy()
        # Solution from https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        m = rotation
        m00 = m[:, 0, 0]
        m01 = m[:, 0, 1]
        m02 = m[:, 0, 2]
        m10 = m[:, 1, 0]
        m11 = m[:, 1, 1]
        m12 = m[:, 1, 2]
        m20 = m[:, 2, 0]
        m21 = m[:, 2, 1]
        m22 = m[:, 2, 2]
        tr = m00 + m11 + m22
        # TODO: it should be possible to matricise the following
        numRotations = m.shape[0]
        qw = np.zeros(numRotations)
        qx = np.zeros(numRotations)
        qy = np.zeros(numRotations)
        qz = np.zeros(numRotations)
        for i in range(m.shape[0]):
            if tr[i] > 0:
                S = np.sqrt(tr[i] + 1.0) * 2
                qw[i] = 0.25 * S
                qx[i] = (m21[i] - m12[i]) / S
                qy[i] = (m02[i] - m20[i]) / S
                qz[i] = (m10[i] - m01[i]) / S
            elif (m00[i] > m11[i]) and(m00[i] > m22[i]):
                S = np.sqrt(1.0 + m00[i] - m11[i] - m22[i]) * 2
                qw[i] = (m21[i] - m12[i]) / S
                qx[i] = 0.25 * S
                qy[i] = (m01[i] + m10[i]) / S
                qz[i] = (m02[i] + m20[i]) / S
            elif m11[i] > m22[i]:
                S = np.sqrt(1.0 + m11[i] - m00[i] - m22[i]) * 2
                qw[i] = (m02[i] - m20[i]) / S
                qx[i] = (m01[i] + m10[i]) / S
                qy[i] = 0.25 * S
                qz[i] = (m12[i] + m21[i]) / S
            else:
                S = np.sqrt(1.0 + m22[i] - m00[i] - m11[i]) * 2
                qw[i] = (m10[i] - m01[i]) / S
                qx[i] = (m02[i] + m20[i]) / S
                qy[i] = (m12[i] + m21[i]) / S
                qz[i] = 0.25 * S
        outDict['rotation'] = np.concatenate((qw[:, np.newaxis],
                                              qx[:, np.newaxis],
                                              qy[:, np.newaxis],
                                              qz[:, np.newaxis],
                                              ), axis=1)
        del outDict['transformation']
        return outDict

def rotToAxisAngle(inDict):
    rotation = inDict['rotation']
    rotationRepresentation = findRotationRepresentation(rotation)
    if rotationRepresentation == 'axisAngle':
        return inDict
    if rotationRepresentation == 'quaternion':
        raise NotImplementedError("Method not implemented")
    elif rotationRepresentation == 'rVec':
        raise NotImplementedError("Method not implemented")
    else: #rotationRepresentation == 'mat'
        raise NotImplementedError("Method not implemented")

'''
Add into the dict two arrays, containing:
(a) the instantaneous linear speed, calculated as the euclidean distance from
  one point to the next / divided by the time from one point to the next,
  using the pairwise average of previous and next to give an estimate for
  each point.
(b) the instantaneous rotational speed, calculated as above expect for angles
'''
def calculateSpeed(inDict):
    def pairwiseMean(array):
        array = array[:, np.newaxis]
        shiftedArray = np.concatenate((array[1:], array[:-1]), axis=1)
        array = np.mean(shiftedArray, axis=1)
        array = np.insert(array, 0, array[0])
        array = np.append(array, array[-1])
        return array
    linearDiffs = np.linalg.norm(np.diff(inDict['point'], axis=0), axis=1)
    timeDiffs = np.diff(inDict['ts'])
    inDict['linearSpeed'] = pairwiseMean(linearDiffs / timeDiffs)
    inDictAsQuat = rotToRepresentation(inDict, 'quaternion')
    rotations = inDictAsQuat['rotation']
    angularDiffs = angleBetweenTwoQuaternionsArray(rotations[:-1, :], rotations[1:, :])
    inDict['angularSpeed'] = pairwiseMean(angularDiffs / timeDiffs)
    return inDict

def interpolatePoses(inDict, times=None, period=None, maxPeriod=None):
    '''
    method: based on the inputs times, period and maxPeriod,
    create a set of times desired
    crop these outside of the range of the existing times
    Then select original times which are also in the new times
    Then do interpolation for each of the new times
    Then merge the two sets.

    It would be possible to create sub-implementations for each rotation
    representation, but for simplicity we convert to quaternion, do the
    interpolation, and convert back again.

    This function is not compatible with labelled poses, or any other
    non-standard additional fields in the poseDict

    '''
    rotation = inDict['rotation']
    rotationRepresentation = findRotationRepresentation(rotation)
    poseDict = rotToQuat(inDict)

    if np.isscalar(times):
        return interpolatePosesSingle(inDict, time=times)

    ts = poseDict['ts']
    points = poseDict['point']
    rotations = poseDict['rotation']

    firstTime = ts[0]
    lastTime = ts[-1]
    if times is not None:
        keepTimesBool = np.logical_and(times >= firstTime, times <= lastTime)
        times = times[keepTimesBool]
        timesAlreadyExistBool = np.isin(times, ts)
        existingTimes = times[timesAlreadyExistBool]
        newTimes = times[~timesAlreadyExistBool]
    elif period is not None:
        times = np.arange(firstTime, lastTime, step=period)
        timesAlreadyExistBool = np.isin(times, ts)
        existingTimes = times[timesAlreadyExistBool]
        newTimes = times[~timesAlreadyExistBool]
    elif maxPeriod is not None:
        proposedAdditionalTimes = np.arange(firstTime, lastTime, maxPeriod)
        prevIds = np.searchsorted(ts, proposedAdditionalTimes, side='right') - 1
        distPre = proposedAdditionalTimes - ts[prevIds]
        distPost = ts[prevIds + 1] - proposedAdditionalTimes
        dist = (distPre + distPost) * (distPre > 0) * (distPost > 0)
        keepAdditional = dist > maxPeriod
        newTimes = proposedAdditionalTimes[keepAdditional]
        existingTimes = ts

    # Now we have newTimes and existingTimes;
    # Now iterate over newTimes and do interpolation
    numNewTimes = len(newTimes)
    newPoints = np.zeros((numNewTimes, 3), dtype=np.float64)
    newRotations = np.zeros((numNewTimes, 4), dtype=np.float64)
    for idx in range(numNewTimes):
        # TODO: slerp function could be vectorised
        # then this whole loop could be vectorised
        newTime = newTimes[idx]
        idxPre = np.searchsorted(ts, newTime, side='right') - 1
        timePre = ts[idxPre]
        timePost = ts[idxPre + 1]
        rotPre = rotations[idxPre, :]
        rotPost = rotations[idxPre + 1, :]
        timeRel = (newTime - timePre) / (timePost - timePre)
        newRotations[idx, :] = slerp(rotPre, rotPost, timeRel)
        pointPre = points[idxPre, :]
        pointPost = points[idxPre + 1, :]
        newPoints[idx, :] = pointPre * (1-timeRel) + pointPost * timeRel

    # get existingTimes for merging
    if maxPeriod is None:
        keepExistingBool = np.isin(ts, existingTimes)
        ts = ts[keepExistingBool]
        points = points[keepExistingBool, :]
        rotations = rotations[keepExistingBool, :]

    ts = np.concatenate((ts, newTimes))
    points = np.concatenate((points, newPoints))
    rotations = np.concatenate((rotations, newRotations))

    newPoseDict = {}
    newPoseDict['ts'] = ts
    newPoseDict['point'] = points
    newPoseDict['rotation'] = rotations
    for key in poseDict.keys():
        if key not in ['ts', 'point', 'rotation']:
            newPoseDict[key] = poseDict[key].copy()
    newPoseDict = sortDataTypeDictByTime(newPoseDict)
    return rotToRepresentation(newPoseDict, rotationRepresentation)

def rotate(inDict, rotation):
    return transform(inDict, rotation=rotation)

def translate(inDict, translation):
    return transform(inDict, translation=translation)

'''
If you pass in transformationDict, it is applied sample-wise as if the dicts
contained mats and the result were: transformationDict @ poseDict
If instead you pass in transformationMatrix, then this is applied to the whole
array thus:
    transformationMatrix @ poseDict
If instead you pass in either translation or rotation, or both, they are
converted into a transformation matrix, and applied to the whole array.
If 'direction' is 'reverse' then the multiplication is instead applied:
    poseDict @ transformationMatrix

The implementation here first converts to transformation matrices
and then converts back at the end
TODO: There could be efficiency gains by not converting to matrices first
'''
def transform(inDict, transformationDict=None, transformationMatrix=None,
              translation=None, rotation=None, direction='normal'):
    rotationRepresentation = findRotationRepresentation(inDict['rotation'])
    inDict = rotToMat(inDict)
    if transformationDict is not None:
        # Assume that the two dicts have exactly the same timestamps
        # TODO: don't assume this but rather interpolate and merge
        # following interpolatePoses function
        transformationDict = rotToMat(transformationDict)
        transformation = transformationDict['transformation']
    else:
        if transformationMatrix is not None:
            transformation = transformationMatrix
        else:
            # Form a transformation matrix from either or both of translation and rotation
            transformation = np.zeros((4, 4), dtype=np.float64)
            transformation[3, 3] = 1
            if translation is not None:
                transformation[:3, 3] = translation
            if rotation is not None:
                rotRepOfTransformation = findRotationRepresentationSingle(rotation)
                if rotRepOfTransformation == 'quaternion':
                    transformation[:3, :3] = quatToMatSingle(rotation,
                                                M=transformation[:3, :3])
                elif rotRepOfTransformation == 'mat':
                    transformation[:3, :3] = rotation
                else:
                    raise NotImplementedError
    if direction in ['normal', 'forward', 'forwards']:
        transformed = transformation @ inDict['transformation']
    elif direction in ['reverse', 'reversed', 'backward', 'backwards']:
        transformed = inDict['transformation'] @ transformation
    else:
        raise ValueError('"direction" argument not recognised')
    outDict = inDict.copy()
    outDict['point'] = transformed[:, :3, 3]
    outDict['rotation'] = transformed[:, :3, :3]
    outDict['transformation'] = transformed

    return rotToRepresentation(outDict, rotationRepresentation)

# rotation is single
# Assuming that rotation comes in as a quaternion; TODO: don't assume this
def angleBetween(inDict, rotation):
    '''
    if rotation.ndim == 1:
    qP = quaternionProduct(q1, quaternionInverse(q2))
    normV = np.linalg.norm(qP[1:])
    angle = 2 * np.arcsin(normV)
    return outDict
    '''
    raise NotImplementedError('not implemented')

def averageRotation(inDict, weights=None):
    # Convert to quaternion
    inDict = rotToMat(inDict)
    rotations = inDict['rotation']
    numRotations = rotations.shape[0]
    if weights is None:
        weights = np.ones(numRotations)
    A = np.zeros((4,4), dtype=np.float64)
    for weight, q in zip(weights, rotations):
        # multiply q with its transposed version q' and accumulate in A
        A = A + weight * np.outer(q,q)
    # scale
    A = A / numRotations
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return eigenVectors[:,0]

"""
A quaternion is in canonical form when it's first component is non-negative.
"""
def canoniseQuaternions(inDict):
    outDict = inDict.copy()
    q = outDict['rotation']
    qw = q[:, 0]
    qSign = qw / np.abs(qw)
    qSign = qSign[:, np.newaxis]
    qCanonised = qSign * q
    outDict['rotation'] = qCanonised
    return outDict

"""
a negative dot product between two quaternions indicates a discontinuity;
pass through the trajectory, flipping at these discontinuities
"""
def makeQuaternionsContinuous(inDict):
    outDict = inDict.copy()
    allQ = outDict['rotation'].copy()
    swapIds = np.where((allQ[:-1, :] * allQ[1:, :]).sum(axis=1) < 0)[0]
    if len(swapIds) % 2 == 1:
        swapIds = np.append(swapIds, (len(allQ) - 1))
    swapBool = np.zeros((len(outDict['ts'])), dtype=np.bool)

    for swapIdsIdx in range(0, len(swapIds), 2):
        swapBool[swapIds[swapIdsIdx]+1 :swapIds[swapIdsIdx + 1]+1] = True
    allQ[swapBool, :] = - allQ[swapBool, :]
    outDict['rotation'] = allQ
    return outDict

'''
Offsets all points so that the first is at zero.
TODO: think of a good way to store the offset, in reference to tsOffset;
If a np[1,3] array is stored, that will break merge and copy dict functionalities.
'''
def zeroPoint(inDict):
    outDict = inDict.copy()
    outDict['point'] = outDict['point'] - outDict['point'][0, :]
    return outDict

'''
rotate all the poses so that the first pose is at [1w, 0x, 0y, 0z]
then offset
'''
def zeroPose(inDict):
    raise NotImplementedError("Method not implemented")


#%% Better names for legacy functions

def interpolatePosesSingle(poseDict, time):
    ts = poseDict['ts']
    points = poseDict['point']
    rotations = poseDict['rotation']
    idxPre = np.searchsorted(ts, time, side='right') - 1
    timePre = ts[idxPre]
    if timePre == time:
        # In this edge-case of desired time == timestamp, there is no need
        # to interpolate
        return (points[idxPre, :], rotations[idxPre, :])
    if idxPre < 0:
        return (points[0, :], rotations[0, :])
    if idxPre >= len(poseDict['ts']):
        return (points[-1, :], rotations[-1, :])
    timePost = ts[idxPre + 1]
    rotPre = rotations[idxPre, :]
    rotPost = rotations[idxPre + 1, :]
    timeRel = (time - timePre) / (timePost - timePre)
    rotOut = slerp(rotPre, rotPost, timeRel)
    pointPre = points[idxPre, :]
    pointPost = points[idxPre + 1, :]
    pointOut = pointPre * (1-timeRel) + pointPost * timeRel
    return (pointOut, rotOut)

# Can accept an existing matrix, which should be min 3x3;
#if it creates a matrix it makes it 4x4
def quatToMatSingle(quat, M=None):
    if M is None:
        M = np.zeros((4, 4))
        M[3, 3] = 1
    elif M.shape[0] == 4:
        M[:, 3] = 0
        M[3, :] = 0
        M[3, 3] = 1
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]
    M[0, 0] = 1 - 2*y**2 - 2*z**2
    M[0, 1] = 2*x*y - 2*z*w
    M[0, 2] = 2*x*z + 2*y*w
    M[1, 0] = 2*x*y + 2*z*w
    M[1, 1] = 1 - 2*x**2 - 2*z**2
    M[1, 2] = 2*y*z - 2*x*w
    M[2, 0] = 2*x*z - 2*y*w
    M[2, 1] = 2*y*z + 2*x*w
    M[2, 2] = 1 - 2*x**2 - 2*y**2
    return M


'''
# following:
#https://stackoverflow.com/questions/24197182/efficient-quaternion-angular-velocity
def deltaRotationAsQuat(angV):
    vectorOfHalfAngle = angV * 0.5
    norm = np.linalg.norm(vectorOfHalfAngle) # magnitude
    if (norm > 0):
        norm *= np.sin(norm) / norm
        return np.insert(vectorOfHalfAngle, 0, np.cos(norm))
    else:
        return np.insert(vectorOfHalfAngle, 0, 1)
'''


'''
A treatment of instantaneous angular velocities from an imu for the purposes
of visualisation
def angularVelocityToRotMat(angV):
    return linalg.expm(np.array(
        [[0, -angV[2], angV[1]],
         [angV[2], 0, -angV[0]],
         [-angV[1], angV[0], 0]])
'''


def pose6qToMatSingle(pose6q):
    T = pose6q['point']
    quat = pose6q['rotation']
    M = quatToMatSingle(quat)
    M[:3,3] = T
    return M


# Spherical linear interpolation, adapted from https://en.wikipedia.org/wiki/Slerp
DOT_THRESHOLD = 0.9995
def slerp(q1, q2, time_relative):
    dot = np.sum(q1 * q2)
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    if dot > DOT_THRESHOLD:
        result = q1 + time_relative * (q2 - q1)
        return (result.T / np.linalg.norm(result)).T
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * time_relative
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0 * q1) + (s1 * q2)

'''
expects pose dict in the form: {'ts': 1d np.array of np.float64 timestamps,
                                'point': 2d array np.float64 of positions [x, y, z],
                                'rotation': 2d array np.float64 of quaternions [rw, rx, ry, rz]
                                (i.e. 6dof with rotation as quaternion)}
Two modes of operation:
If time is not None, then returns the interpolated pose at that time -
    returns (point, rotation) tuple, being np.array 1d x 3 and 4 respectively, np.float64,    which is interpolated pose;
Else if maxPeriod is not None, then it returns the entire pose dict,
    but with additional points necessary to ensure that time between samples
    never exceeds maxPeriod
'''

def quaternionProductSingle(q1, q2):
    return np.array([
    q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
    q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
    q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
    q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0] ])
    ''' alternative definition - less efficient
    w1 = q1[0]
    v1 = q1[1:4]
    w2 = q2[0]
    v2 = q2[1:4]
    wOut = w1*w2 - np.dot(v1.T, v2)
    vOut = w1*v2 + w2*v1 + np.cross(v1, v2)
    qOut = np.zeros_like(q1)
    qOut[0] = wOut
    qOut[1:4] = vOut
    return qOut
    '''

# Array version of the above
def quaternionProductArray(q1, q2):
    def x(q, idx): # tis function to clean up the following expression
        return q[:, idx, np.newaxis]
    return np.concatenate((
    x(q1,0) * x(q2,0) - x(q1,1) * x(q2,1) - x(q1,2) * x(q2,2) - x(q1,3) * x(q2,3),
    x(q1,0) * x(q2,1) + x(q1,1) * x(q2,0) + x(q1,2) * x(q2,3) - x(q1,3) * x(q2,2),
    x(q1,0) * x(q2,2) - x(q1,1) * x(q2,3) + x(q1,2) * x(q2,0) + x(q1,3) * x(q2,1),
    x(q1,0) * x(q2,3) + x(q1,1) * x(q2,2) - x(q1,2) * x(q2,1) + x(q1,3) * x(q2,0)
    ), axis=1)

def quaternionConjugateSingle(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quaternionConjugateArray(q):
    return np.concatenate((q[:, 0, np.newaxis], -q[:, 1:4]), axis=1)

def quaternionInverseSingle(q):
    return quaternionConjugateSingle(q) / np.sum(q ** 2)

def quaternionInverseArray(q):
    return quaternionConjugateArray(q) / np.sum(q ** 2, axis=1)[:, np.newaxis]

def angleBetweenTwoQuaternionsSingle(q1, q2):
    # returns minimal angle in radians between two unit quaternions, following:
    # https://www.researchgate.net/post/How_do_I_calculate_the_smallest_angle_between_two_quaternions
    qP = quaternionProductSingle(q1, quaternionInverseSingle(q2))
    normV = np.linalg.norm(qP[1:])
    return 2 * np.arcsin(normV)

def angleBetweenTwoQuaternionsArray(q1, q2):
    qP = quaternionProductArray(q1, quaternionInverseArray(q2))
    normV = np.linalg.norm(qP[:, 1:], axis=1)
    return 2 * np.arcsin(normV)

'''
following https://github.com/KieranWynn/pyquaternion/blob/master/pyquaternion/quaternion.py
'''
def axisAngleToQuatSingle(axis, angle):
    mag_sq = np.dot(axis, axis)
    if mag_sq == 0.0:
        raise ZeroDivisionError("Provided rotation axis has no length")
    # Ensure axis is in unit vector form
    if (abs(1.0 - mag_sq) > 1e-12):
        axis = axis / np.sqrt(mag_sq)
    theta = angle / 2.0
    r = np.cos(theta)
    i = axis * np.sin(theta)
    quat = np.concatenate((np.ones((1)) * r, i))
    quat = quat / np.linalg.norm(quat)
    return quat

'''
following: https://answers.unity.com/questions/1266985/convert-rotation-vector-or-matrix-to-quaternion.html
Receives a rotation vector and returns a quaternion
'''
def rVecToQuatSingle(rotV):
    theta = np.linalg.norm(rotV)
    quat = axisAngleToQuatSingle(rotV[:, 0], theta)
    return quat

'''
Expects
    - poseDict in bimvee form {'ts', 'point', 'rotation' as above}
    - translation as np array of x,y,z
    - rotation as a np array of w,x,y,z (quaternion)
If either translation or rotation are passed, these are applied to all poses
in the poseDict.
The rotations are defined wrt local axes, unlike the translations.
Returns a copy of the poseDict, rotated
'''
def transformPosesOld(poseDict, translation=None, rotation=None):
    # Create a copy of the input array - use the same contents
    outDict = {}
    for key in poseDict.keys():
        outDict[key] = poseDict[key]
    if translation is not None:
        outDict['point'] = poseDict['point'] + translation
    if rotation is not None:
        for idx in range(outDict['rotation'].shape[0]): # TODO: this could be matricised
            outDict['rotation'][idx, :] = \
                quaternionProductSingle(outDict['rotation'][idx, :], rotation)
    return outDict


# The following adapted from https://github.com/christophhagen/averaging-quaternions
# Note that the signs of the output quaternion can be reversed,
# since q and -q describe the same orientation.
# w is an optional weight array, which must have the same number of elements
# as the number of quaternions
def averageOfQuaternions(allQ, w=None):
    # Number of quaternions to average
    M = allQ.shape[0]
    A = np.zeros((4,4), dtype = np.float64)
    if w is None:
        w = np.ones(M,)
    weightSum = 0
    for i in range(0, M):
        q = allQ[i, :]
        # multiply q with its transposed version q' and add A
        A = w[i] * np.outer(q,q) + A
        weightSum += w[i]
    # scale
    A = (1.0 / M) * A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0])

'''
iitVicon can produce consecutive pose samples with same ts, point and rotation
eliminate these
Ignore different bodyIds for now
'''
def removeDuplicates(poseDict):
    ts = poseDict['ts']
    point = poseDict['point']
    rotation = poseDict['rotation']
    tsDiff = np.diff(ts)
    pointDiff = np.sum(np.diff(point, axis=0), axis=1)
    rotationDiff = np.sum(np.diff(rotation, axis=0), axis=1)
    toKeep = np.logical_or(tsDiff!=0, pointDiff!=0, rotationDiff!=0)
    # remove following elements by shifting toKeep dict forwards
    toKeep = np.insert(toKeep, 0, 'True')
    return selectByBool(poseDict, toKeep)


