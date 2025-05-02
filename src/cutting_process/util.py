#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Various utility functions
'''

import os
import random
import numpy
import pb_robot
from tsr.tsr import TSR, TSRChain

class VanishBody(object): # todo
    def __init__(self, body):
        self.body = body
    def simulate(self):
        self.body.remove_body()
    def getBody(self):
        return self.body
    def updateBody(self, body):
        self.body = body
    def execute(self, realRobot=None):
       # dictPath = [realRobot.convertToDict(q) for q in self.path]
       # realRobot.execute_position_path(dictPath)
       pass #todo
    def __repr__(self):
        return 'vanish{}'.format(id(self) % 1000)
    
class CreateHalves(object): # todo
    def __init__(self, h1_path, h2_path, h1_transform, h2_transform):
        self.h1_path = h1_path
        self.h2_path = h2_path
        self.h1_transform = h1_transform
        self.h2_transform = h2_transform
        self.h1_name = h1_path[:-len(".urdf")]
        self.h2_name = h2_path[:-len(".urdf")]
        self.body1 = None
        self.body2 = None
    def simulate(self):
        curr_path = os.getcwd()
        models_path = os.path.join(os.path.dirname(curr_path), 'models')

        h1_file = os.path.join(models_path, self.h1_path)
        potato1 = pb_robot.body.createBody(h1_file)
        potato1.set_transform(self.h1_transform)

        h2_file = os.path.join(models_path, self.h2_path)
        potato2 = pb_robot.body.createBody(h2_file)
        potato2.set_transform(self.h2_transform)

        self.body1 = potato1
        self.body2 = potato2
        # todo we need references to these objects in main loop

    def execute(self, realRobot=None):
       # dictPath = [realRobot.convertToDict(q) for q in self.path]
       # realRobot.execute_position_path(dictPath)
       pass #todo
    def __repr__(self):
        return 'createHalves{}'.format(id(self) % 1000) # todo what is going on here

class CreatePile(object):
    def __init__(self, transform):
        self.transform = transform
    def simulate(self):
        curr_path = os.getcwd()
        models_path = os.path.join(os.path.dirname(curr_path), 'models')

        # TODO pile not same potato
        
        potato_file = os.path.join(models_path, 'cucumber.urdf')
        potato = pb_robot.body.createBody(potato_file)

        potato.set_transform(self.transform)
    def execute(self, realRobot=None):
       pass
    def __repr__(self):
        return 'createPile{}'.format(id(self) % 1000)

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

phantom_bodies_map = {}

def dephantomize(obj):
    if type(obj) == VanishBody and type(obj.getBody()) == type("string"):
        phantom = obj.getBody()
        obj.updateBody(phantom_bodies_map[phantom])

# dephantomize -- TODO
'''print(plan)
for action in plan:
    dephantomized = []
    for arg in action.args:
        dephantomized.append(samplers.dephantomize(arg))
    action._replace(args=tuple(dephantomized))
print(plan)'''

def ExecuteActions(plan):
    '''Iterate through the plan, simulating each action'''
    for name, args in plan:
        # preprocessing yay
        pb_robot.viz.remove_all_debug()
        bodyNames = [args[i].get_name() for i in range(len(args)) if isinstance(args[i], pb_robot.body.Body)]
        txt = '{} - {}'.format(name, bodyNames)
        pb_robot.viz.add_text(txt, position=(0, 0.25, 0.5), size=2)

        # don't do these things for actions that don't do robot stuff
        # make things magically appear and disappear instead
        print(name)
        print(args)
        
        executionItems = args[-1] # is this the trajectory?
        # trajectory can include things that are not movements?
        # or is trajectory just always the last one?
        #if name != "slice_move": #fix fix
        for e in executionItems:
            print(e)
            if type(e) == VanishBody:
                dephantomize(e)
            print(e)
            # todo handle dephantomizing here as needed
            e.simulate()
            if type(e) == CreateHalves:
                phantom_bodies_map[e.h1_name] = e.body1
                phantom_bodies_map[e.h2_name] = e.body2
            # do i want to remove things from phantom body map as they are vanished?
            input("Next?")
