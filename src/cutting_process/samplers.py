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
import xml.etree.ElementTree as ET

phantom_bodies_map = {}
dedupe_num = 0

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

def split_object(object, ancestor, pose, knife): ## can get o's attribs? can add predicates here? aaaa
    # todo return actual potatoes?
    # is this possible? is this necessary?

    # some dummy obj that you can do body ops on but isn't in the world yet
    # 'python class for pile' <-- can mimic from vobj.py as template

    # todo -- make a note about reserved keywords // filenames // body names

    # does this need deduped with a counter so that each body can have multiple phantom halves?
    # todo add dynamic generation logic
    # todo change from hardcoded to urdf parsing (OR from pb_robot?)

    global dedupe_num

    if type(object) != type("string"):
        # this is the original object!
        filename = "./../models/cuttable.urdf"
    else:
        filename = phantom_bodies_map[object][1]
    # Parse the URDF XML
    tree = ET.parse(filename)
    root = tree.getroot()

    # todo edit robot name to include size?
    length = 0.25

    # Find all <box> elements anywhere in the tree
    for box in root.findall('.//box'):
        size_attr = box.get('size')
        if size_attr:
            # Parse the three dimensions
            try:
                sizes = [float(val) for val in size_attr.strip().split()]
                if len(sizes) == 3:
                    # Halve the Y dimension
                    length = sizes[1]
                    sizes[1] = sizes[1] / 2.0
                    # Format back to string (preserve 6 decimal places)
                    box.set('size', f"{sizes[0]:.6f} {sizes[1]:.6f} {sizes[2]:.6f}")
            except ValueError:
                # Skip if parsing fails
                continue

    # Prepare output filename
    base, ext = os.path.splitext(filename)
    name_h1 = base + "_h1_" + str(dedupe_num)
    name_h2 = base + "_h2_" + str(dedupe_num)

    dedupe_num += 1

    # Write the modified URDF
    tree.write(name_h1 + ext, encoding='utf-8', xml_declaration=True)
    tree.write(name_h2 + ext, encoding='utf-8', xml_declaration=True)

    distance = 0.001

    h1_transform = pose.pose.copy()
    h1_transform[0][3] -= length / 4 + distance / 2

    h2_transform = pose.pose.copy()
    h2_transform[0][3] += length / 4 + distance / 2

    print("ARE THESE DIFFERENT???????????")
    print(h1_transform)
    print(h2_transform)

    h1_pose = pb_robot.vobj.BodyPose(name_h1, h1_transform)
    h2_pose = pb_robot.vobj.BodyPose(name_h2, h2_transform)

    phantom_bodies_map[name_h1] = [h1_transform, name_h1 + ext]
    phantom_bodies_map[name_h2] = [h2_transform, name_h2 + ext]

    down_wrench1 = pb_robot.vobj.BodyWrench(name_h1, [0, 0, -3, 0, 0, 0])
    across_wrench1 = pb_robot.vobj.BodyWrench(name_h1, [0.5, 0, -0.1, 0, 0, 0])
    down_wrench2 = pb_robot.vobj.BodyWrench(name_h2, [0, 0, -3, 0, 0, 0])
    across_wrench2 = pb_robot.vobj.BodyWrench(name_h2, [0.5, 0, -0.1, 0, 0, 0])

    return (name_h1, name_h2, h1_pose, h2_pose, down_wrench1, across_wrench1, down_wrench2, across_wrench2, [cutting_process.util.VanishBody(object), cutting_process.util.CreateHalves(name_h1 + ext, name_h2 + ext, h1_transform, h2_transform)])
    # as long as diff things are returned here these are treated as diff
    # since these are generated, pddl can't be stupid and pick the same half for both

def make_pile_from(o):
    # todo dephantom
    # todo return actual pile
    return (None, [cutting_process.util.VanishBody(o), cutting_process.util.CreatePile(o.get_transform())])

def pose_collision_test(o1, p1, o2, p2):
    '''Check if object o1 (at pose p1) is in collision with object o2 (at pose p2)'''
    vaporize_o1 = False
    vaporize_o2 = False
    if type(o1) == type("string"):
        vaporize_o1 = True
        curr_path = os.getcwd()
        models_path = os.path.join(os.path.dirname(curr_path), 'models')
        o1_file = os.path.join(models_path, phantom_bodies_map[o1][1])
        o1 = pb_robot.body.createBody(o1_file)
    else:
        o1_og = o1.get_transform()
    if type(o2) == type("string"):
        vaporize_o2 = True
        curr_path = os.getcwd()
        models_path = os.path.join(os.path.dirname(curr_path), 'models')
        o2_file = os.path.join(models_path, phantom_bodies_map[o2][1])
        o2 = pb_robot.body.createBody(o2_file)
    else:
        o2_og = o2.get_transform()

    o1.set_transform(p1.pose)
    o2.set_transform(p2.pose)
    collision = pb_robot.collisions.pairwise_collision(o1, o2)

    if vaporize_o1:
        o1.remove_body()
    else:
        o1.set_transform(o1_og)
    if vaporize_o2:
        o2.remove_body()
    else:
        o2.set_transform(o2_og)
    return not collision

def traj_collision_test(arm, traj, obj, pose):
    '''Tries to certify if collision-free. Return false if not collision free'''
    vaporize_obj = False
    if type(obj) == type("string"):
        vaporize_obj = True
        curr_path = os.getcwd()
        models_path = os.path.join(os.path.dirname(curr_path), 'models')
        obj_file = os.path.join(models_path, phantom_bodies_map[obj][1])
        obj = pb_robot.body.createBody(obj_file)
    else:
        obj_og = obj.get_transform()
    obj.set_transform(pose.pose)

    for q in (traj[0]).path:
        arm.SetJointValues(q)
        inCollision = not arm.IsCollisionFree(q)
        if inCollision:
            if vaporize_obj:
                obj.remove_body()
            else:
                obj.set_transform(obj_og)
            return False
    if vaporize_obj:
        obj.remove_body()
    else:
        obj.set_transform(obj_og)
    return True


def exists_ik(arm, grasp):
    '''Check if a grasp is kinematically reachable'''
    for _ in range(5):
        ik = arm.ComputeIK(grasp) # grasp here is not a vobj
        if ik is not None and arm.IsCollisionFree(ik):
            return True
    return False  

def get_grasp_gen():
    '''Generate function for sampling grasps'''
    def gen(arm, body):
        '''Sample collision-free reachable grasps for body, using arm'''
        vaporize_body = False
        original_body = body
        if type(body) == type("string"):
            vaporize_body = True
            curr_path = os.getcwd()
            models_path = os.path.join(os.path.dirname(curr_path), 'models')
            body_file = os.path.join(models_path, phantom_bodies_map[body][1])
            body = pb_robot.body.createBody(body_file) # todo do i need to set pose here?
        for _ in range(10):
            grasp_worldF = cutting_process.generators.getObjRelations(arm, body) # this return cannot be phantom
            # Collision check
            if not exists_ik(arm, grasp_worldF): 
                continue
            grasp_objF = numpy.dot(numpy.linalg.inv(body.get_transform()), grasp_worldF) # frame conversion math! (worldF --> objF)
            radius = cutting_process.mechanics.computeRadius(body, grasp_objF)
            body_grasp = pb_robot.vobj.BodyGrasp(original_body, grasp_objF, arm, r=radius,
                                                 mu=cutting_process.mechanics.lookupMu(arm, body))
            if vaporize_body:
                body.remove_body()
            return (body_grasp,)
        if vaporize_body:
            body.remove_body()
    return gen

def get_stable_gen(fixed=[]):
    '''Generate function for sampling placement poses'''
    def gen(body, surface): # surface cannot currently be a phantom (as per stackable predicate)
        '''Sample collision-free poses for body on a surface'''
        vaporize_body = False
        original_body = body
        if type(body) == type("string"):
            vaporize_body = True
            curr_path = os.getcwd()
            models_path = os.path.join(os.path.dirname(curr_path), 'models')
            body_file = os.path.join(models_path, phantom_bodies_map[body][1])
            body = pb_robot.body.createBody(body_file) # todo do i need to set pose here?
        for _ in range(5):
            pose = cutting_process.generators.getObjRelations(surface, body)
            if any(pb_robot.collisions.pairwise_collision(body, b) for b in fixed):
                continue
            body_pose = pb_robot.vobj.BodyPose(original_body, pose)
            if vaporize_body:
                body.remove_body()
            return (body_pose,)
        if vaporize_body:
            body.remove_body()
        return None 
    return gen

def get_ik_fn(fixed=None, num_attempts=10):
    '''Generate function for solving ik for grasping and placing'''
    def fn(arm, body, pose, grasp): # i don't think this needs to be dephantomized?
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

def get_free_motion_gen(fixed=None): # no dephantomization
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

def get_holding_motion_gen(fixed=None): # todo not sure if this is correct? if this leaves objects where they should be how is picking up objects experimented with? 
    '''Generate function for planning holding paths, accounting for fixed obstacles'''
    def fn(arm, conf1, conf2, body, grasp):
        '''For arm, plan a collision-free path from conf1 to conf2, while holding body with grasp'''
        vaporize_body = False
        vaporize_graspBody = False
        if type(body) == type("string"):
            vaporize_body = True
            curr_path = os.getcwd()
            models_path = os.path.join(os.path.dirname(curr_path), 'models')
            body_file = os.path.join(models_path, phantom_bodies_map[body][1])
            body = pb_robot.body.createBody(body_file) # todo do i need to set pose here?
        if type(grasp.body) == type("string"):
            vaporize_graspBody = True
            curr_path = os.getcwd()
            models_path = os.path.join(os.path.dirname(curr_path), 'models')
            graspBody_file = os.path.join(models_path, phantom_bodies_map[grasp.body][1])
            graspBody = pb_robot.body.createBody(graspBody_file) # todo do i need to set pose here?
            grasp = pb_robot.vobj.BodyGrasp(graspBody, grasp.grasp_objF, grasp.manip, grasp.r, grasp.mu, grasp.N) # assuming no ViseGrasps
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
                if vaporize_body:
                    body.remove_body()
                if vaporize_graspBody:
                    grasp.body.remove_body()
                return None

        command = [pb_robot.vobj.JointSpacePath(arm, path)]
        if vaporize_body:
            body.remove_body()
        if vaporize_graspBody:
            grasp.body.remove_body()
        return (command,)
    return fn

#####################################################################


def test_grasp_stability(arm, obj, wrench, grasp):
    '''Evaluate if grasp is stable with respect to wrench'''
    vaporize_graspBody = False
    vaporize_wrenchBody = False
    if type(grasp.body) == type("string"):
        vaporize_graspBody = True
        curr_path = os.getcwd()
        models_path = os.path.join(os.path.dirname(curr_path), 'models')
        graspBody_file = os.path.join(models_path, phantom_bodies_map[grasp.body][1])
        graspBody = pb_robot.body.createBody(graspBody_file)
        graspBody.set_transform(phantom_bodies_map[grasp.body][0]) # where the thing actually is in sim is used in the mechanics fn?
        grasp = pb_robot.vobj.BodyGrasp(graspBody, grasp.grasp_objF, grasp.manip, grasp.r, grasp.mu, grasp.N) # assuming no ViseGrasps
    if type(wrench.body) == type("string"):
        vaporize_wrenchBody = True
        curr_path = os.getcwd()
        models_path = os.path.join(os.path.dirname(curr_path), 'models')
        wrenchBody_file = os.path.join(models_path, phantom_bodies_map[wrench.body][1])
        wrenchBody = pb_robot.body.createBody(wrenchBody_file)
        wrenchBody.set_transform(phantom_bodies_map[wrench.body][0]) # where the thing actually is in sim is used in the mechanics fn?
        wrench = pb_robot.vobj.BodyWrench(wrenchBody, wrench.ft_objF)
    to_return =  cutting_process.mechanics.checkGraspStability(arm, grasp, wrench, checkToolContact=False)
    if vaporize_graspBody:
        grasp.body.remove_body()
    if vaporize_wrenchBody:
        wrench.body.remove_body()
    return to_return

def get_stable_grasp_gen():
    '''Generate function for sampling stable grasps'''
    def gen(arm, body, wrench):
        '''Sample collision-free reachable grasps for body, using arm. Must be stable with respect to wrench'''
        vaporize_body = False
        original_body = body
        vaporize_wrenchBody = False
        if type(body) == type("string"):
            vaporize_body = True
            curr_path = os.getcwd()
            models_path = os.path.join(os.path.dirname(curr_path), 'models')
            body_file = os.path.join(models_path, phantom_bodies_map[body][1])
            body = pb_robot.body.createBody(body_file) # todo do i need to set pose here?
        if type(wrench.body) == type("string"):
            vaporize_wrenchBody = True
            curr_path = os.getcwd()
            models_path = os.path.join(os.path.dirname(curr_path), 'models')
            wrenchBody_file = os.path.join(models_path, phantom_bodies_map[wrench.body][1])
            wrenchBody = pb_robot.body.createBody(wrenchBody_file) # todo do i need to set pose here?
            wrench = pb_robot.vobj.BodyWrench(wrenchBody, wrench.ft_objF)
        try:
            if body.get_name() not in wrench.body.get_name():
                check = cutting_process.generators.getObjRelations(body, wrench.body)
        except NotImplementedError:
            # Using the wrong wrench!
            if vaporize_body:
                body.remove_body()
            if vaporize_wrenchBody:
                wrench.body.remove_body()
            return None

        for _ in range(20):
            grasp_worldF = cutting_process.generators.getObjRelations(arm, body)
            # Collision check
            if not exists_ik(arm, grasp_worldF):
                continue
            grasp_objF = numpy.dot(numpy.linalg.inv(body.get_transform()), grasp_worldF)
            radius = cutting_process.mechanics.computeRadius(body, grasp_objF)
            body_grasp = pb_robot.vobj.BodyGrasp(original_body, grasp_objF, arm, r=radius,
                                                 mu=cutting_process.mechanics.lookupMu(arm, body))

            stability = cutting_process.mechanics.checkGraspStability(arm, body_grasp, wrench)
            if stability:
                if vaporize_body:
                    body.remove_body()
                if vaporize_wrenchBody:
                    wrench.body.remove_body()
                return (body_grasp,)
            else:
                continue
        if vaporize_body:
            body.remove_body()
        if vaporize_wrenchBody:
            wrench.body.remove_body()
        return None
    return gen


def slice_cut(fixed=[]): # todo LOL the knife does need to come back up
    '''Generate a function for planning the slicing motion'''
    def fn(arm, knife, obj, grasp, obj_pose, w0, w1): # obj_pose is where planner wants it to be -- preconditions; obj.pose may be different because out of order things (while planning pybullet world != pddl world)
        '''For arm, use knife (held by grasp) to cut obj (located at obj_pose) by slicing with wrenches w1 and w2'''
        vaporize_obj = False
        vaporize_w0Body = False
        vaporize_w1Body = False
        if type(obj) == type("string"):
            vaporize_obj = True
            curr_path = os.getcwd()
            models_path = os.path.join(os.path.dirname(curr_path), 'models')
            obj_file = os.path.join(models_path, phantom_bodies_map[obj][1])
            obj = pb_robot.body.createBody(obj_file)
        if type(w0.body) == type("string"):
            vaporize_w0Body = True
            curr_path = os.getcwd()
            models_path = os.path.join(os.path.dirname(curr_path), 'models')
            w0Body_file = os.path.join(models_path, phantom_bodies_map[w0.body][1])
            w0Body = pb_robot.body.createBody(w0Body_file) # todo do i need to set pose here?
            w0 = pb_robot.vobj.BodyWrench(w0Body, w0.ft_objF)
        if type(w1.body) == type("string"):
            vaporize_w1Body = True
            curr_path = os.getcwd()
            models_path = os.path.join(os.path.dirname(curr_path), 'models')
            w1Body_file = os.path.join(models_path, phantom_bodies_map[w1.body][1])
            w1Body = pb_robot.body.createBody(w1Body_file) # todo do i need to set pose here?
            w1 = pb_robot.vobj.BodyWrench(w1Body, w1.ft_objF)
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
            conf_end = pb_robot.vobj.BodyConf(arm, q_preStartFollow)
            #pb_robot.arm.SetJointValues(q_startFollow0)

            command = [pb_robot.vobj.MoveToTouch(arm, q_preStartFollow, q_startFollow0),
                       pb_robot.vobj.CartImpedPath(arm, start_q=q_startFollow0, ee_path=cart_hand_path0, stiffness=stiffness0),
                       #pb_robot.vobj.CartImpedPath(arm, start_q=q_startFollow1, ee_path=cart_hand_path1, stiffness=stiffness1),
                       pb_robot.vobj.MoveFromTouch(arm, q_preStartFollow)
                       ]
            pb_robot.viz.remove_all_debug()
            
            # todo

            if vaporize_obj:
                obj.remove_body()
            if vaporize_w0Body:
                w0.body.remove_body()
            if vaporize_w1Body:
                w1.body.remove_body()
            return (conf_start, conf_end, command,) # returns end config too!
        if vaporize_obj:
            obj.remove_body()
        if vaporize_w0Body:
            w0.body.remove_body()
        if vaporize_w1Body:
            w1.body.remove_body()
        return None
    return fn

# todo
# todo vapor and dephantomize operations. i'm too tired right now
def dice_cut(fixed=[]):
    '''Generate a function for planning the dicing motion'''
    def fn(arm, knife, obj, grasp, obj_pose, w):
        obj = dephantomize(obj) # ELEPHANT
        '''For arm, use knife (held by grasp) to cut obj (located at obj_pose) by slicing with wrenches w1 and w2'''

        for _ in range(5): 

            mating_worldF = cutting_process.generators.getObjRelations(knife, obj, body2_pose=obj_pose.pose)
            mating_objF = numpy.dot(numpy.linalg.inv(obj_pose.pose), mating_worldF)

            obj_aabb = pb_robot.aabb.get_aabb(obj)
            height = pb_robot.aabb.get_aabb_extent(obj_aabb)[2]
            resulting0 = cutting_process.control.generateCartPathFromWrench(w, obj_pose.pose, dist=1.6*height) 
            if resulting0 is None: continue 
            (path_thru_obj0, start_obj_pose0, stiffness0) = resulting0
            tool_path0 = cutting_process.util.actionPath_hand(path_thru_obj0, mating_objF)
            cart_hand_path0 = cutting_process.util.actionPath_hand(tool_path0, grasp.grasp_objF)

            approxJointPath0 = cutting_process.control.findStartIK(arm, cart_hand_path0, fixed=fixed)
            if approxJointPath0 is None:
                if DEBUG_FAILURE: print('Cant Find first Cartesian Path')
                continue 
       
            start_tool_pose0 = numpy.dot(start_obj_pose0, mating_objF)
            start_pose0 = numpy.dot(start_tool_pose0, grasp.grasp_objF)
            q_startFollow0 = arm.ComputeIK(start_pose0, seed_q=approxJointPath0[0])
            if q_startFollow0 is None:
                if DEBUG_FAILURE: print('No IK: q_startfollow0')
                continue 

            q_preStartFollow = backInKin(arm, q_startFollow0, [0, 0.03, 0], grasp, fixed)
            q_postEndFollow = backOutKin(arm, approxJointPath0[-1], [0, 0.03, 0], grasp, fixed)
            if q_preStartFollow is None or q_postEndFollow is None: 
                continue 

            # Package up all variables to return
            conf_start = pb_robot.vobj.BodyConf(arm, q_preStartFollow)
            conf_end = conf_start
            #conf_end = pb_robot.vobj.BodyConf(arm, q_postEndFollow)
            #pb_robot.arm.SetJointValues(q_startFollow0)

            command = [pb_robot.vobj.MoveToTouch(arm, q_preStartFollow, q_startFollow0),
                       pb_robot.vobj.CartImpedPath(arm, start_q=q_startFollow0, ee_path=cart_hand_path0, stiffness=stiffness0),
                       pb_robot.vobj.MoveFromTouch(arm, q_preStartFollow),

                       pb_robot.vobj.MoveToTouch(arm, q_preStartFollow, q_startFollow0),
                       pb_robot.vobj.CartImpedPath(arm, start_q=q_startFollow0, ee_path=cart_hand_path0, stiffness=stiffness0),
                       pb_robot.vobj.MoveFromTouch(arm, q_preStartFollow)
                       ]
            pb_robot.viz.remove_all_debug()

            return (conf_start, conf_end, command,) # returns end config too!
        return None
    return fn