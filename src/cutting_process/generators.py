#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Functions needed to (object, object) relations, i.e. defines grasping and placing sets
'''

import numpy
from tsr.tsr import TSR, TSRChain
import pb_robot 
import cutting_process

def getObjRelations(body1, body2, body2_pose=None):
    '''For a pair of objects, generate the sampler for their relation'''
    tsr = None
    if body2_pose is None:
        body2_pose = body2.get_transform()

    if 'panda' in body1.get_name():
        # Grasping
        if 'knife' in body2.get_name():
            tsr = pb_robot.tsrs.panda_tool_handle.handle_grasp(body2)
        elif 'potato' in body2.get_name():
            tsr = pb_robot.tsrs.panda_box.grasp(body2) 
        else:
            raise NotImplementedError("No grasp set defined for {}".format(body2.get_name()))

    if 'knife' in body1.get_name() and 'potato' in body2.get_name():
        tsr = matingPositions_knifeCut(body1, body2, body2_pose)
    if 'holder' in body1.get_name() and 'knife' in body2.get_name():
        tsr = matingPositions_knifeHolder(body1, body2, body2_pose)

    if tsr is None:
        print("No relation defined for ({}, {}). Using default placement sampler".format(body1.get_name(), body2.get_name()))
        pose = pb_robot.placements.sample_placement(body2, body1)
        return pb_robot.geometry.tform_from_pose(pose)

    # For now all of these are TSRs so return a sample
    return cutting_process.util.SampleTSRForPose(tsr)

###############################################################################
### All the generators! 
###############################################################################

def matingPositions_knifeCut(knife, obj, obj_pose):
    '''Generate feasible starting locations for the knife to cut the obj'''
    obj_aabb = pb_robot.aabb.get_aabb(obj)
    width = (pb_robot.aabb.get_aabb_extent(obj_aabb)[0] / 2.0) - 0.01
    height = pb_robot.aabb.get_aabb_extent(obj_aabb)[2] -0.003
    T0_w = obj_pose

    Tw_e_right = numpy.array([[1., 0., 0., 0.15],
                              [0., 0, -1., 0.],
                              [0., 1., 0., height], 
                              [0., 0., 0., 1.]])
    Bw_right = numpy.zeros((6, 2))
    Bw_right[0, :] = [-0.001, 0.001]
    Bw_right[1, :] = [-width/2.0, width/2.0]
    Bw_right[1, :] = [-0.001, 0.001]
    Bw_right[2, :] = [-0.001, 0.001]
    right_tsr = TSR(T0_w=T0_w, Tw_e=Tw_e_right, Bw=Bw_right)
    right_chain = TSRChain(sample_start=False, sample_goal=True,
                           constrain=False, TSR=right_tsr)
    return [right_chain] 

def matingPositions_knifeHolder(knife, holder, holder_pose):
    '''Generate feasible poses for the knife to be in the knife holder'''
    obj_aabb = pb_robot.aabb.get_aabb(holder)
    width = (pb_robot.aabb.get_aabb_extent(obj_aabb)[0] / 2.0) - 0.01
    T0_w = holder_pose
    Tw_e = numpy.array([[1., 0., 0., 0.01],
                        [0., 1., 0., 0.0],
                        [0., 0., 1., 0.0],
                        [0., 0., 0., 1.]])             
    Bw = numpy.zeros((6, 2))
    Bw[0, :] = [-0.005, 0.005]
    Bw[1, :] = [-0.001, 0.001]
    Bw[2, :] = [-0.001, 0.001] 
    tsr_holder = TSR(T0_w=T0_w, Tw_e=Tw_e, Bw=Bw)
    chain_holder = TSRChain(sample_start=False, sample_goal=True,
                            constrain=False, TSR=tsr_holder) 
    return [chain_holder]
