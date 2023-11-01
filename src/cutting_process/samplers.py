#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Samplers needed for PDDLStream. Implements grasping, placing, planning, cutting and collision-checking
'''


import copy
import numpy
import pb_robot
import cutting_process

import os

DEBUG_FAILURE = False

def backInKin(arm, q_start, direction_in, grasp=None, fixed=[]):
    '''Compute the configuration needed to "back-in" to a motion.
    If grasp is not none then you are holding a tool'''
    pose_start = arm.ComputeFK(q_start)
    if grasp is None:
        pose_preStart = cutting_process.util.ComputePrePose(pose_start, direction_in)
    else:
        pose_toolStart = numpy.dot(pose_start, numpy.linalg.inv(grasp.grasp_objF))
        pose_preStart = cutting_process.util.ComputePrePose(pose_toolStart, direction_in, grasp.grasp_objF)

    q_preStart = arm.ComputeIK(pose_preStart, seed_q=q_start)
    if q_preStart is None or not arm.IsCollisionFree(q_preStart, obstacles=fixed):
        return None

    return q_preStart

def backOutKin(arm, q_end, direction_out, grasp=None, fixed=[]):
    '''Compute the configuration needed to "back-out" of a motion.
    If grasp is not none then you are holding a tool'''
    pose_end = arm.ComputeFK(q_end)
    if grasp is None:
        pose_postEnd = cutting_process.util.ComputePrePose(pose_end, direction_out)
    else:
        pose_toolEnd = numpy.dot(pose_end, numpy.linalg.inv(grasp.grasp_objF))
        pose_postEnd = cutting_process.util.ComputePrePose(pose_toolEnd, direction_out, grasp.grasp_objF)
    q_postEnd = arm.ComputeIK(pose_postEnd, seed_q=q_end)
    if q_postEnd is None or not arm.IsCollisionFree(q_postEnd, obstacles=fixed):
        return None

    return q_postEnd

#####################################################################

def split_object(o): ## can get o's attribs? can add predicates here? aaaa
    # todo return actual potatoes?
    # is this possible? is this necessary?
    return (None, None, [cutting_process.util.VanishBody(o), cutting_process.util.CreateHalves(o.get_transform())])

def make_pile_from(o):
    # todo return actual pile?
    # is this possible? is this necessary?
    return (None, [cutting_process.util.VanishBody(o), cutting_process.util.CreatePile(o.get_transform())])

def pose_collision_test(o1, p1, o2, p2):
    '''Check if object o1 (at pose p1) is in collision with object o2 (at pose p2)'''
    o1_og = o1.get_transform()
    o2_og = o2.get_transform()

    o1.set_transform(p1.pose)
    o2.set_transform(p2.pose)
    collision = pb_robot.collisions.pairwise_collision(o1, o2)

    o1.set_transform(o1_og)
    o2.set_transform(o2_og)
    return not collision

def traj_collision_test(arm, traj, obj, pose):
    '''Tries to certify if collision-free. Return false if not collision free'''
    obj_og = obj.get_transform()
    obj.set_transform(pose.pose)

    for q in (traj[0]).path:
        arm.SetJointValues(q)
        inCollision = not arm.IsCollisionFree(q)
        if inCollision:
            obj.set_transform(obj_og)
            return False

    obj.set_transform(obj_og)
    return True


def exists_ik(arm, grasp):
    '''Check if a grasp is kinematically reachable'''
    for _ in range(5):
        ik = arm.ComputeIK(grasp)
        if ik is not None and arm.IsCollisionFree(ik):
            return True
    return False  

def get_grasp_gen():
    '''Generate function for sampling grasps'''
    def gen(arm, body):
        '''Sample collision-free reachable grasps for body, using arm'''
        for _ in range(10):
            grasp_worldF = cutting_process.generators.getObjRelations(arm, body)
            # Collision check
            if not exists_ik(arm, grasp_worldF): 
                continue
            grasp_objF = numpy.dot(numpy.linalg.inv(body.get_transform()), grasp_worldF)
            radius = cutting_process.mechanics.computeRadius(body, grasp_objF)
            body_grasp = pb_robot.vobj.BodyGrasp(body, grasp_objF, arm, r=radius,
                                                 mu=cutting_process.mechanics.lookupMu(arm, body))
            return (body_grasp,)
    return gen

def get_stable_gen(fixed=[]):
    '''Generate function for sampling placement poses'''
    def gen(body, surface):
        '''Sample collision-free poses for body on a surface'''
        for _ in range(5):
            pose = cutting_process.generators.getObjRelations(surface, body)
            if any(pb_robot.collisions.pairwise_collision(body, b) for b in fixed):
                continue
            body_pose = pb_robot.vobj.BodyPose(body, pose)
            return (body_pose,)
        return None 
    return gen

def get_ik_fn(fixed=None, num_attempts=10):
    '''Generate function for solving ik for grasping and placing'''
    def fn(arm, body, pose, grasp):
        '''For arm, plan the configuration and path needed to grasp/ungrasp a body at pose with grasp'''
        obj_worldF = pose.pose
        grasp_worldF = numpy.dot(obj_worldF, grasp.grasp_objF)

        for i in range(num_attempts):
            # Sample approaching from many different directions
            if i % 4 < 2: # First two times (bias towards z)
                approach_dir = [0, 0, -0.15]
            elif i % 4 == 2: 
                approach_dir = [0, -0.15, 0]
            else: 
                approach_dir = [-0.15, 0, 0]
            approach_tform = cutting_process.util.ComputePrePose(grasp_worldF, approach_dir)
            q_approach = arm.ComputeIK(approach_tform)
            if (q_approach is None): 
                if DEBUG_FAILURE: print("Q approach failed")
                continue
            q_grasp = arm.ComputeIK(grasp_worldF, seed_q=q_approach)
            if (q_grasp is None):
                if DEBUG_FAILURE: print("Q grasp failed")
                continue

            command = [pb_robot.vobj.MoveToTouch(arm, q_approach, q_grasp), grasp, # think lights
                       pb_robot.vobj.MoveToTouch(arm, q_grasp, q_approach)]
            conf = pb_robot.vobj.BodyConf(arm, q_approach)
            return (conf, command)
        return None
    return fn

def get_free_motion_gen(fixed=None):
    '''Generate function for planning free paths, accounting for fixed obstacles'''
    def fn(arm, conf1, conf2):
        '''For arm, plan a collision-free path from conf1 to conf2'''
        path = arm.birrt.PlanToConfiguration(arm, conf1.configuration, conf2.configuration, obstacles=fixed)
        if path is None:
            if DEBUG_FAILURE: print('Free motion failed')
            return None
        command = [pb_robot.vobj.JointSpacePath(arm, path)]
        return (command,)
    return fn

def get_holding_motion_gen(fixed=None):
    '''Generate function for planning holding paths, accounting for fixed obstacles'''
    def fn(arm, conf1, conf2, body, grasp):
        '''For arm, plan a collision-free path from conf1 to conf2, while holding body with grasp'''
        arm.Grab(grasp.body, grasp.grasp_objF)
        path = arm.birrt.PlanToConfiguration(arm, conf1.configuration, conf2.configuration, obstacles=fixed)
        arm.Release(grasp.body)
        if path is None:
            if DEBUG_FAILURE: print('Holding motion failed')
            return None

        # For now, check stability outside planner.
        wrench = pb_robot.vobj.BodyWrench(body, [0, 0, 0, 0, 0, 0]) # Hence only checks gravity
        for q in path:
            if not cutting_process.mechanics.checkArmStability(arm, grasp, q, wrench, gravity=True):
                if DEBUG_FAILURE: print('Cartesian Path not stable')
                return None

        command = [pb_robot.vobj.JointSpacePath(arm, path)]
        return (command,)
    return fn

#####################################################################


def test_grasp_stability(arm, obj, wrench, grasp):
    '''Evaluate if grasp is stable with respect to wrench'''
    return cutting_process.mechanics.checkGraspStability(arm, grasp, wrench, checkToolContact=False)

def get_stable_grasp_gen():
    '''Generate function for sampling stable grasps'''
    def gen(arm, body, wrench):
        '''Sample collision-free reachable grasps for body, using arm. Must be stable with respect to wrench'''
        try:
            if body.get_name() not in wrench.body.get_name():
                check = cutting_process.generators.getObjRelations(body, wrench.body)
        except NotImplementedError:
            # Using the wrong wrench!
            return None

        for _ in range(20):
            grasp_worldF = cutting_process.generators.getObjRelations(arm, body)
            # Collision check
            if not exists_ik(arm, grasp_worldF):
                continue
            grasp_objF = numpy.dot(numpy.linalg.inv(body.get_transform()), grasp_worldF)
            radius = cutting_process.mechanics.computeRadius(body, grasp_objF)
            body_grasp = pb_robot.vobj.BodyGrasp(body, grasp_objF, arm, r=radius,
                                                 mu=cutting_process.mechanics.lookupMu(arm, body))

            stability = cutting_process.mechanics.checkGraspStability(arm, body_grasp, wrench)
            if stability:
                return (body_grasp,)
            else:
                continue
        return None
    return gen


def slice_cut(fixed=[]):
    '''Generate a function for planning the slicing motion'''
    def fn(arm, knife, obj, grasp, obj_pose, w0, w1): # obj_pose is where planner wants it to be -- preconditions; obj.pose may be different because out of order things (while planning pybullet world != pddl world)
        '''For arm, use knife (held by grasp) to cut obj (located at obj_pose) by slicing with wrenches w1 and w2'''

        for _ in range(5): 

            mating_worldF = cutting_process.generators.getObjRelations(knife, obj, body2_pose=obj_pose.pose)
            mating_objF = numpy.dot(numpy.linalg.inv(obj_pose.pose), mating_worldF)

            ###################
            # First Wrench Path
            ###################
            obj_aabb = pb_robot.aabb.get_aabb(obj)
            height = pb_robot.aabb.get_aabb_extent(obj_aabb)[2]
            resulting0 = cutting_process.control.generateCartPathFromWrench(w0, obj_pose.pose, dist=1.6*height) 
            if resulting0 is None: continue 
            (path_thru_obj0, start_obj_pose0, stiffness0) = resulting0
            tool_path0 = cutting_process.util.actionPath_hand(path_thru_obj0, mating_objF)
            cart_hand_path0 = cutting_process.util.actionPath_hand(tool_path0, grasp.grasp_objF)

            approxJointPath0 = cutting_process.control.findStartIK(arm, cart_hand_path0, fixed=fixed)
            if approxJointPath0 is None:
                if DEBUG_FAILURE: print('Cant Find first Cartesian Path')
                continue 

            ####################
            # Second Wrench Path
            ####################
            resulting1 = cutting_process.control.generateCartPathFromWrench(w1, path_thru_obj0[-1])
            if resulting1 is None: continue 
            (path_thru_obj1, start_obj_pose1, stiffness1) = resulting1
            tool_path1 = cutting_process.util.actionPath_hand(path_thru_obj1, mating_objF)
            cart_hand_path1 = cutting_process.util.actionPath_hand(tool_path1, grasp.grasp_objF)

            # Here the starting ik is already determined by the previous path
            approxJointPath1 = cutting_process.control.checkPathIK(arm, cart_hand_path1, approxJointPath0[-1], fixed=fixed)
            if approxJointPath1 is None:
                if DEBUG_FAILURE: print('Cant Find second Cartesian Path')
                continue
       
            start_tool_pose0 = numpy.dot(start_obj_pose0, mating_objF)
            start_pose0 = numpy.dot(start_tool_pose0, grasp.grasp_objF)
            q_startFollow0 = arm.ComputeIK(start_pose0, seed_q=approxJointPath0[0])
            if q_startFollow0 is None:
                if DEBUG_FAILURE: print('No IK: q_startfollow0')
                continue 

            seed_q = copy.deepcopy(q_startFollow0)
            for i in range(len(cart_hand_path0)):
                seed_q = arm.ComputeIK(cart_hand_path0[i], seed_q=seed_q)
            
            start_tool_pose1 = numpy.dot(start_obj_pose1, mating_objF)
            start_pose1 = numpy.dot(start_tool_pose1, grasp.grasp_objF)
            q_startFollow1 = arm.ComputeIK(start_pose1, seed_q=seed_q)
            if q_startFollow1 is None:
                if DEBUG_FAILURE: print('No IK: q_startfollow1')
                continue 

            q_preStartFollow = backInKin(arm, q_startFollow0, [0, 0.03, 0], grasp, fixed)
            q_postEndFollow = backOutKin(arm, approxJointPath1[-1], [0.03, 0, 0], grasp, fixed)
            if q_preStartFollow is None or q_postEndFollow is None: 
                continue 

            # Package up all variables to return
            conf_start = pb_robot.vobj.BodyConf(arm, q_preStartFollow)
            conf_end = pb_robot.vobj.BodyConf(arm, q_postEndFollow)
            #pb_robot.arm.SetJointValues(q_startFollow0)

            command = [pb_robot.vobj.MoveToTouch(arm, q_preStartFollow, q_startFollow0),
                       pb_robot.vobj.CartImpedPath(arm, start_q=q_startFollow0, ee_path=cart_hand_path0, stiffness=stiffness0), 
                       #pb_robot.vobj.CartImpedPath(arm, start_q=q_startFollow1, ee_path=cart_hand_path1, stiffness=stiffness1),
                       #pb_robot.vobj.MoveFromTouch(arm, q_postEndFollow)
                       ]
            pb_robot.viz.remove_all_debug()

            return (conf_start, q_startFollow1, command,) # returns end config too!
        return None
    return fn
