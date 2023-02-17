#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Various utility functions
'''

import random
import numpy
import pb_robot
from tsr.tsr import TSR, TSRChain

def SampleTSRForPose(tsr_chain):
    '''Shortcutting function for randomly samping a 
    pose from a tsr chain
    @param tsr_chain Chain to sample from
    @return ee_pose Pose from tsr chain'''
    tsr_idx = random.randint(0, len(tsr_chain)-1)
    sampled_tsr = tsr_chain[tsr_idx]
    ee_pose = sampled_tsr.sample()
    return ee_pose

def CreateTSRFromPose(pose):
    '''Create a TSR that, when sampled, produces one pose. This 
    simply creates a common interface for the planner
    @param manip Manipulator to use use (required for tsr)
    @param pose 4x4 transform to center TSR on
    @return tsr_chain chain with single pose TSR'''
    goal_tsr = TSR(T0_w=pose)
    tsr_chain = TSRChain(sample_goal=True, TSR=goal_tsr)
    return tsr_chain

def actionPath_hand(tool_path, grasp_toolF):
    '''Given a series of waypoints of the tool in the world frame we
    create a path for the hand in the world frame by transforming 
    using the grasp
    @param tool_path Series of waypoints of tool in world frame
    @param grasp_toolF Pose of the end effector when grasping in the tool frame
    @return hand_pose Series of waypoints of the hand in the world frame'''
    return numpy.array([numpy.dot(tool_path[i], grasp_toolF) for i in range(len(tool_path))])

def ComputePrePose(og_pose, directionVector, relation=None):
    '''Given a pose, compute the "backed-up" pose, i.e. the pose offset
    by the desired direction vector
    @param og_pose Transform to offset from
    @param directionVector 3D vector to offset by
    @param relation Optional parameter corresponding to a grasp
    @return prepose Transform of og_pose offset by directionVector'''
    backup = numpy.eye(4)
    backup[0:3, 3] = directionVector
    prepose = numpy.dot(og_pose, backup)
    if relation is not None:
        prepose = numpy.dot(prepose, relation)
    return prepose

def get_fixed(movable):
    '''Given the set of movable bodies in a scene, the fixed bodies are defined
    to be any other body in the scene'''
    movable_ids = [m.id for m in movable]
    fixed = [body for body in pb_robot.utils.get_bodies() if body.id not in movable_ids]
    return fixed

def ExecuteActions(plan):
    '''Iterate through the plan, simulating each action'''
    for name, args in plan:
        pb_robot.viz.remove_all_debug()
        bodyNames = [args[i].get_name() for i in range(len(args)) if isinstance(args[i], pb_robot.body.Body)]
        txt = '{} - {}'.format(name, bodyNames)
        pb_robot.viz.add_text(txt, position=(0, 0.25, 0.5), size=2)

        executionItems = args[-1]
        for e in executionItems:
            print(e)
            e.simulate()
            input("Next?")
