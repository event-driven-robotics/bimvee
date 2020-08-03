# -*- coding: utf-8 -*-
"""
Created on Thu Jul 30 10:51:24 2020

@author: sbamford
"""

containerRealsense is just the pose6q
containerVicon pose6q

from bimvee.pose import interpolatePoses, rotToMat, quatToMatSingle

# find what is the first common time and last common time
times = np.arange(fct, lct)
containerRealsenseAligned = interpolatePoses(containerRealsense, times=times)
containerViconAligned = interpolatePoses(containerVicon, times=times)

#paramsToOptimise = np.array((7), dtype=np.float64)
initialGuess = np.array((7), dtype=np.float64)

'''
[x, y, z, qw, qx, qy, qz]
'''

containerRealsenseAlignedMat = rotToMat(containerRealsenseAligned)

realsenseTransformationMats = containerRealsenseAlignedMat['transformation']

viconPoints = containerViconAligned['point']


def residualFunc(coeff):
    ripAll = []
    q = coeff[3:]
    q = q / np.linalg.norm(q)
    transformationMat = quatToMatSingle(q)
    transformationMat[:3, 3] = coeff[:3]

    # for loop to ddo the following
    transformedMats = transformationMat @ realsenseTransformationMats
    transformedPoints = transformedMats[:, :3, 3]

    loss =  np.sum(np.sqrt((transformedPoints[:, 0] - viconPoints[:, 0])**2 +
                           (transformedPoints[:, 1] - viconPoints[:, 1])**2 +
                           (transformedPoints[:, 2] - viconPoints[:, 2])**2))

    return loss

optResult = scipy.optimize.least_squares(residualFunc,
                                         initialGuess)
# Now normalise the output quaternion

# Now apply that result to ContainerRealsense

= realsenseAlignedInSpace

#%%

'''
2nd part of problem, find a rotation that pointwise aligns realsenseAlignedInSpace to containerVicon
Or in Leandro case:
find a rotation that aligns 
