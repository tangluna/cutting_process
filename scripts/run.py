#!/usr/bin/env python

from __future__ import print_function

import time
import numpy
import IPython
import pb_robot
import cutting_process 

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_fn, from_test 
from pddlstream.utils import read, INF 
from pddlstream.language.constants import print_solution
from pddlstream.language.stream import StreamInfo

def pddlstream_from_problem(robots, movable):
    '''Create the pieces needed for a PDDLStream problem, including definign
    the initial set of facts and the streams'''
    domain_pddl = read('domain.pddl') 
    stream_pddl = read('stream.pddl') 
    constant_map = {}

    print('Robot:', robots)
    init = [('CanMove',)]
    for r in robots:
        conf = pb_robot.vobj.BodyConf(r.arm, r.arm.GetJointValues())
        init += [('HandEmpty', r.arm), 
                 ('Arm', r.arm),
                 ('Conf', conf), 
                 ('AtConf', r.arm, conf),
                 ('InWorld', r.arm)]

    fixed = cutting_process.util.get_fixed(robots+movable) 
    print('Movable:', [m.get_name() for m in movable])
    print('Fixed:', [f.get_name() for f in fixed])
    for body in movable:
        pose = pb_robot.vobj.BodyPose(body, body.get_transform())
        init += [('Graspable', body),
                 ('Pose', body, pose),
                 ('AtPose', body, pose), 
                 ('Movable', body),
                 ('InWorld',  body)]

        if 'potato' in body.get_name():
            init += [('Cuttable', body)]

        if 'knife' in body.get_name():
            init += [('Knife', body)]

        for surface in fixed: 
            if 'potato' in body.get_name() and 'table' in surface.get_name():
                init += [('Stackable', body, surface)]
                #init += [('Supported', body, pose, surface)]
            if 'knife' in body.get_name() and 'holder' in surface.get_name():
                init += [('Stackable', body, surface)]
                #init += [('Supported', body, pose, surface)]
            if 'knife' in body.get_name() and 'table' in surface.get_name():
                init += [('Stackable', body, surface)]
                init += [('Supported', body, pose, surface)]

    for surface in fixed:
        init += [('InWorld', surface)]
        if 'table' in surface.get_name():
            init += [('Region', surface)]
        if 'holder' in surface.get_name():
            init += [('Region', surface)]

    down_wrench = pb_robot.vobj.BodyWrench(movable[0], [0, 0, -3, 0, 0, 0])
    across_wrench = pb_robot.vobj.BodyWrench(movable[0], [0.5, 0, -0.1, 0, 0, 0])
    init += [('Wrench', down_wrench), ('Wrench', across_wrench)]
    init += [('SliceCutWrenches', movable[1], movable[0], down_wrench, across_wrench)]
 
    goal = ('and', ('SlicesInWorld', movable[0]))
    # is the goal that something is sliced or that we have a sliced piece? (one piece or two -- currently 2)
    # stacking this is gonna be hard w/o defining more predicates

    #goal = ('and', ('On', movable[0], fixed[0]))
    #goal state is derived predicate!

    stream_map = {
        'sample-pose': from_fn(cutting_process.samplers.get_stable_gen(fixed)),
        'sample-grasp': from_fn(cutting_process.samplers.get_grasp_gen()),
        'inverse-kinematics': from_fn(cutting_process.samplers.get_ik_fn(fixed)), 
        'plan-free-motion': from_fn(cutting_process.samplers.get_free_motion_gen(fixed)),
        'plan-holding-motion': from_fn(cutting_process.samplers.get_holding_motion_gen(fixed)),
        'sample-force-grasp': from_fn(cutting_process.samplers.get_stable_grasp_gen()),
        'plan-slice-cut-motion': from_fn(cutting_process.samplers.slice_cut(fixed)),
        'test-pose-cfree': from_test(cutting_process.samplers.pose_collision_test),
        'test-traj-cfree': from_test(cutting_process.samplers.traj_collision_test),
        'test-grasp-stable': from_test(cutting_process.samplers.test_grasp_stability),
        'generate-cut-objects': from_fn(cutting_process.samplers.split_object),
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

#######################################################

if __name__ == '__main__':
    pb_robot.utils.connect(use_gui=True)
    pb_robot.utils.set_default_camera()
    robots, movable = cutting_process.environments.cuttingTableScene()

    saved_world = pb_robot.utils.WorldSaver()
    pddlstream_problem = pddlstream_from_problem(robots, movable)
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', stream_map.keys())

    stream_info = {'test-pose-cfree': StreamInfo(negate=True),
                   'test-traj-cfree': StreamInfo(negate=True)}

    IPython.embed()
    start_time = time.time()
    solution = solve_focused(pddlstream_problem, stream_info=stream_info,
                             planner='ff-astar', unit_costs=True, success_cost=numpy.inf, 
                             max_time=INF, debug=False)
    end_time = time.time()

    print_solution(solution)
    plan, cost, evaluations = solution
    print('Time ', end_time-start_time)
    print('\n')
 
    if plan is None:
        print("No plan found")
    else:
        saved_world.restore()
        input("Execute?")
        # this can be commented out! (line below)
        cutting_process.util.ExecuteActions(plan)
        IPython.embed()

    input('Finish?')
    pb_robot.utils.disconnect()
