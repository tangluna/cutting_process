#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Functions for computing impedance control related aspects 
NOTE: This file is particularly not well documented
'''

import numpy

def findStartIK(arm, hand_path, attempts=3, fixed=[]):
    '''Evaluate if exists reachable set of configurations for a starting configuration value'''
    for _ in range(attempts):
        start_ik = arm.ComputeIK(hand_path[0])
        if start_ik is None or not arm.IsCollisionFree(start_ik, obstacles=fixed):
            continue

        path = checkPathIK(arm, hand_path, start_ik, fixed)
        if path is None:
            continue
        else:
            return path

    return None

def checkPathIK(arm, hand_path, next_ik, fixed=[]):
    '''Evaluate if feasible path exists'''
    jpath = [next_ik]
    for i in range(1, len(hand_path)):
        next_ik = arm.ComputeIK(hand_path[i], seed_q=next_ik) 
        if next_ik is None or not arm.IsCollisionFree(next_ik, obstacles=fixed):
            return None 
        else: 
            jpath += [next_ik]
    return jpath

def getCartImpedanceOffset(desiredForce):
    ''' Best Fit mapping from data. Which stiffness to use?
    Stiffness 400:  f = 426d - 2.39
    Stiffness 200:  f = 231d - 1.87
    Stiffness 100:  f = 124d - 1.23
    Stiffness 50:   f = 79.3d - 0.99
    Stiffness 0:    f = 50.7d - 0
    '''
    if desiredForce > 0: 
        return (None, None)
    if desiredForce < -20:
        return (None, None) # Can exert more then 20 - faults
    s0 = (desiredForce) / 50.7
    s50 = (desiredForce + 0.99) / 79.3
    s100 = (desiredForce + 1.23) / 124.0
    s200 = (desiredForce + 1.87) / 231.0
    s400 = (desiredForce + 2.39) / 426.0
    stiffness = [0, 50, 100, 200, 400]

    distances = numpy.array([s0, s50, s100, s200, s400])
    selection = numpy.argmax(distances[distances <= 0])
    base_stiffness = stiffness[selection]
    stiffness6D = [base_stiffness]*6
    stiffness6D[3:6] = numpy.divide(stiffness6D[3:6], 10)

    return (distances[selection], stiffness6D)


def generateCartPathFromWrench(wrench, obj_tform, dist=0.1, n=3):
    '''
    - get rotation directions (only allow 0 or 1). 
    - get linear directions (no limit on how many). 
    - if exists rotation direction - that determines the motion. 
        any linear directions control down_dist + stiffness in that direction
    - if only linear directions, take the largest magnitude linear. that determines motion
        any other linear directions control down_dist + stiffness in that direction

    - directions with motion: high stiffness. 
    - if direction not given by motion or force exertion - assume high stiffness..?
    '''
    linearDirs = numpy.nonzero(wrench.ft_objF[0:3])[0]
    rotationDirs = numpy.nonzero(wrench.ft_objF[3:6])[0]

    # Generate path based on largest force
    incMove = numpy.eye(4)
    motionLinearDir = numpy.argmax(map(abs, wrench.ft_objF[0:3]))
    incMove[motionLinearDir, 3] = (dist/n)*numpy.sign(wrench.ft_objF[motionLinearDir])
    # Remove this direction so it isnt used to generate "force"
    linearDirs = linearDirs[linearDirs != motionLinearDir]

    # Generate path based on "motion" piece of wrench
    motionPath = numpy.zeros((n+1, 4, 4))
    motionPath[0, :, :] = obj_tform
    for i in range(n):
        obj_tform = numpy.dot(obj_tform, incMove)
        motionPath[i+1, :, :] = obj_tform
    firstPoint = motionPath[0]

    # Add any "force" piece of the wrench
    if len(linearDirs) == 0:
        # No "force" component. Use high stiffness
        stiffness = [1200, 1200, 1200, 40, 40, 40]
        return (motionPath, firstPoint, stiffness)
    else:
        for ldir in linearDirs:
            # In "force" directions
            (down_dist, stiffness) = getCartImpedanceOffset(wrench.ft_objF[ldir])
            if down_dist is None: return None
            factor = numpy.zeros((4, 4))
            factor[ldir, 3] = down_dist
            motionPath = motionPath + factor

    return (motionPath, firstPoint, stiffness)
