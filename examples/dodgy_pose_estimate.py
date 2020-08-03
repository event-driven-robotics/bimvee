#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 30 11:31:33 2020

@author: leandro
"""
#
a_pose = vicon['pose']
b_pose = real_sense['pose'] # or IMU

interpolate(a, b, time) # time alignment

#maybe have to invert the rotation matrices and/or add a minus in the point
a_space_aligned_points = a_pose['rotation'] @ a_pose['point']
b_space_aligned_points = b_pose['rotation'] @ b_pose['point']

#-----------------------------------------------------------------------------
# this is second part of the problem
def residualFunc(M, pa, pb):
    return np.linalg.norm(pa - M @ pb)

result = scypi.optimize.square(residualFunc, initialGuess)


#-----------------------------------------------------------------------------
def residualFunc(M, pa, pb):
    prod  = kroneker_matrix(a_space_aligned_points , b_space_aligned_points)
    s, sigma, u = sdv(prod)

    R = s*u.T
    T = a-R*b

    residual = pa*[Ra Ta]-[Rb Tb]*pb

result = scypi.optimize.square(residualFunc, initialGuess)
