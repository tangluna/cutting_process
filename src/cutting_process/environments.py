#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Functions needed to create the simulation scenes
'''

import os
import numpy
import pb_robot
import pybullet

def cuttingTableScene():
    '''Creating a scene with a robot, table, knife, knife holder, and
    and object to cut. The object to cut is on a cutting board'''
    robotA = pb_robot.panda.Panda()
    robotA.set_point([0, 0, -0.1])
    robotA.arm.hand.Open()
    robotList = [robotA]

    pybullet.configureDebugVisualizer(lightPosition=[2, 0, 3])
    pybullet.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-20, cameraTargetPosition=[0, 0, 0])
    curr_path = os.getcwd()
    models_path = os.path.join(os.path.dirname(curr_path), 'models')

    # adding things to pybullet ref! file, body, spot
    table_file = os.path.join(models_path, 'panda_table.urdf')
    table = pb_robot.body.createBody(table_file)
    table.set_point([0.2, 0, -0.12])

    knife_file = os.path.join(models_path, 'knife_metal.urdf')
    knife = pb_robot.body.createBody(knife_file)
    
    knife_pose = numpy.array([[0., 0., 1., 0.05],
                              [1., 0., 0., 0.4], 
                              [0., -1., 0., 0.1],
                              [0., 0., 0., 1.]])
    knife.set_transform(knife_pose)

    knifeholder_file = os.path.join(models_path, 'knife_holder_metal.urdf')
    knifeholder = pb_robot.body.createBody(knifeholder_file)
    knifeholder_pose = numpy.array([[0., 0., 1., knife_pose[0, 3]],
                                    [0., 1., 0., knife_pose[1, 3]],
                                    [1., 0., 0., knife_pose[2, 3]-0.2],  
                                    [0., 0., 0., 1.]])
    knifeholder.set_transform(knifeholder_pose)

    board_file = os.path.join(models_path, 'cutting_board.urdf')
    board = pb_robot.body.createBody(board_file)
    board_xyz = [0.42, 0.0, pb_robot.placements.stable_z(board, table)] 
    board.set_point(board_xyz)

    potato_file = os.path.join(models_path, 'cucumber.urdf')
    potato = pb_robot.body.createBody(potato_file)
    potato_pose = numpy.array([[0., -1., 0., board_xyz[0]],
                               [1., 0., 0., board_xyz[1]],
                               [0., 0., 1., pb_robot.placements.stable_z(potato, board)+0.005], 
                               [0., 0., 0., 1.]])
    potato.set_transform(potato_pose)
    movable = [potato, knife]

    return robotList, movable
