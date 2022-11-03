#usr/bin/env python
# -*- coding: utf-8 -*-

'''
Various mechanics functions for forceful manipulation 
'''

import numpy
import pb_robot
import cutting_process

def lookupMu(obj1_name, obj2_name):
    if not isinstance(obj1_name, str):
        obj1_name = obj1_name.get_name()[:-1]
    if not isinstance(obj2_name, str):
        obj2_name = obj2_name.get_name()[:-1]

    if 'panda' in obj1_name:
        if 'knife' in obj2_name: return 0.6
        elif 'potato' in obj2_name: return 0.8
        else: raise NotImplementedError("No mu value for panda and {}".format(obj2_name))
    elif 'hand' in obj1_name:
        if 'potato' in obj2_name: return 0.8
        else: raise NotImplementedError("No mu value for hand and {}".format(obj2_name))

    # Other
    elif 'knife' in obj1_name and 'potato' in obj2_name: return 0.5
    else:
        raise NotImplementedError("Do not have a specified mu value for interaction between {} and {}".format(obj1_name, obj2_name))

def computeRadius(grasped_obj, grasp_objF):
    if 'knife' in grasped_obj.get_name():
        ee_to_finger = numpy.eye(4)
        ee_to_finger[2, 3] = 0.1
        finger_objF = numpy.dot(grasp_objF, ee_to_finger)
        finger_y = finger_objF[2, 3]

        maxwidth = 0.02
        maxwiggleroom = 0.005
        if abs(finger_y) <= maxwiggleroom: radius_y = maxwidth
        else: radius_y = maxwidth - (abs(finger_y) - maxwiggleroom)
        return max(radius_y, 0)
    # Default size
    else:
        return 0.02
   
def isStableGrasp(extForces, mu=0.5, r=0.02, N=40):
    '''Is the grasp stable under the external forces?
    Original Finger: r=0.007, mu=0.4
    @param extForce (list) Planar forces of the form [fx, fz, ty]
    @param mu (float) friction at fingers
    @param r (float) radius of contact (M)
    @param N (float) grasp force (N) 
    @return (boolean) Check if grasp is stable'''
    c = 0.6 # Constant, found by lynch

    # Extract Individual Forces these forces
    fx_bar = extForces[0]
    fz_bar = extForces[2]
    my_bar = extForces[4]

    # Recompute grip force 
    eps = 1e-3
    if (N == 0) or (r == 0) or (mu == 0):
        if fx_bar > eps or fz_bar > eps or my_bar > eps: return False
        else: return True

    force = (fx_bar**2 / (2*mu*N)**2) + (fz_bar**2 / (2*mu*N)**2) + (my_bar**2 / (((2*mu*N)**2)*((r*c)**2)))
    return force < 1 # If true, stable grasp. else, unstable grasp

def getContactFrame(graspPose):
    '''
    Define the contact frame, which is given as an fixed offset from the base
    of the end effector. This preserves this frame, i.e. if graspPose is in 
    world frame this returns the contact frame in the world frame. If graspPose
    is given in the object frame, what we return is also in the object frame. 
    @param graspPose transform of the hand
    @return contactFrame
    '''
    #gives in same frame as grasp pose so doesnt change frames
    contact_offset = numpy.eye(4)
    contact_offset[2, 3] = 0.1
    contactF = numpy.dot(graspPose, contact_offset) 
    return contactF
 
def skewMatrix(w):
    '''
    Create a 3x3 skew symmetric matrix from 3D vector w
    @param w vector to use
    @return 3x3 skew symmetric matrix
    ''' 
    skewW = numpy.array([[0, -w[2], w[1]],
                         [w[2], 0, -w[0]],
                         [-w[1], w[0], 0]])  
    return skewW

def adjointTransform(T):
    '''
    From a transform T in SE(3), create the adjoint representation following
    from Lynch and Park's book
    @param T a homogenous transformation in SE(3)
    @param adj 6x6 matrix that is adjoint representation of T
    '''
    # Extract Rotation and Transformation
    R = T[0:3, 0:3] 
    p = T[0:3, 3]

    # Compute adjoint representation - Definition 3.20 (Page 100)
    adj = numpy.zeros((6, 6))
    adj[0:3, 0:3] = R
    adj[3:6, 0:3] = numpy.dot(skewMatrix(p), R)
    adj[3:6, 3:6] = R
    return adj

def wrenchFrameTransformation(w, frameA, frameB):
    '''Given a wrench: [f_x, f_y, f_z, t_x, t_y, t_z] move it from frameA to
    frameB. Uses Prop 3.27 from Lynch and Parks book for coordination
    transformation with wrenches. Frame a and frame b must be with respect to
    the same reference frame.
    @param w wrench in frame A
    @param frameA transform representing frameA in the reference frame
    @param frameB transform representing frameB in the reference frame
    @return newW wrench in frame B'''
    # Compute transform from frame A to frame B
    T = numpy.dot(frameB, numpy.linalg.inv(frameA))

    # Lynch Prop 3.27 to move wrenches (Pg 110) 
    #newW = numpy.dot(numpy.transpose(adjointTransform(T)), w) # Used lynch examples to confirm computation
    #return newW 

    # In lynch representation its [torque, force]. So swap order, perform operation, swap back.
    tf = [w[3], w[4], w[5], w[0], w[1], w[2]]
    tf_new = numpy.dot(numpy.transpose(adjointTransform(T)), tf)
    w_new = numpy.array([tf_new[3], tf_new[4], tf_new[5], tf_new[0], tf_new[1], tf_new[2]])
    return w_new


def checkGraspStability(arm, grasp, wrench, gravity=False, checkToolContact=True):
    if wrench.body == grasp.body:
        # Direct Grasp
        stable = stability_direct(arm, grasp, wrench)
    else:
        # Indirect grasp. Sample relation between grasping body and body where wrench is defined
        for _ in range(5):
            mating_worldF = cutting_process.generators.getObjRelations(grasp.body, wrench.body, wrench.body.get_transform())
            mating_objF = numpy.dot(numpy.linalg.inv(wrench.body.get_transform()), mating_worldF)
            grasp_objF = numpy.dot(mating_objF, grasp.grasp_objF)  #Also ee_objF

            # Package up grasp_objF and mating_objF
            body_grasp = pb_robot.vobj.BodyGrasp(grasp.body, grasp_objF, arm, r=grasp.r, mu=grasp.mu)

            body_mating = pb_robot.vobj.BodyGrasp(wrench.body, mating_objF, grasp.body, 
                                                  mu=lookupMu(grasp.body, wrench.body))
           
            stable = stability_indirect(arm, body_grasp, body_mating, wrench, checkToolContact=checkToolContact)
            if stable: return stable

    return stable

def checkArmStability(arm, grasp, q, wrench, gravity=False, checkToolContact=True):
    if wrench.body == grasp.body:
        # Direct Grasp
        stable = stability_direct(arm, grasp, wrench, q=q)
    else:
        # Indirect grasp. Need to compute grasp_objF and mating_objF
        inmating_grasp_worldF = arm.ComputeFK(q)
        mating_worldF = numpy.dot(inmating_grasp_worldF, numpy.linalg.inv(grasp.grasp_objF))
        mating_objF = numpy.dot(numpy.linalg.inv(wrench.body.get_transform()), mating_worldF)
        grasp_objF = numpy.dot(mating_objF, grasp.grasp_objF)  #Also ee_objF
        # Package up grasp_objF and mating_objF
        body_grasp = pb_robot.vobj.BodyGrasp(grasp.body, grasp_objF, arm, r=grasp.r, mu=grasp.mu)

        body_mating = pb_robot.vobj.BodyGrasp(wrench.body, mating_objF, grasp.body, 
                                              mu=lookupMu(grasp.body, wrench.body))

        #stable = stability_indirect(arm, body, grasp_objF, mating_objF, wrench, q=q)
        stable = stability_indirect(arm, body_grasp, body_mating, wrench, q=q, checkToolContact=checkToolContact)
    
    return stable

def stability_direct(arm, grasp, wrench, q=None, gravity=True):
    '''Grasp is on obj'''
    obj_objF = numpy.eye(4) 
    worldF_worldF = numpy.eye(4)
    worldF_worldF[0:3, 3] = grasp.body.get_transform()[0:3, 3]

    gravity_worldF = numpy.array([0, 0, -9.8*grasp.body.get_mass(), 0, 0, 0]) # cog = com 
    gravity_objF = wrenchFrameTransformation(gravity_worldF, worldF_worldF, wrench.body.get_transform())

    rcontact_objF = getContactFrame(grasp.grasp_objF)
    wrench_rcontactF = wrenchFrameTransformation(wrench.ft_objF, obj_objF, rcontact_objF)
    if gravity:
        gravity_rcontactF = wrenchFrameTransformation(gravity_objF, obj_objF, rcontact_objF)
        wrench_rcontactF += gravity_rcontactF

    rgrasp_stability = isStableGrasp(wrench_rcontactF, mu=grasp.mu, r=grasp.r) 

    if q is None:
        #print('Grasp: {}'.format(rgrasp_stability))
        return rgrasp_stability
    else:
        # grasp_objF is also eeF_objF
        wrench_eeF = wrenchFrameTransformation(wrench_rcontactF, grasp.grasp_objF, rcontact_objF)
        if gravity:
            gravity_eeF = wrenchFrameTransformation(gravity_objF, obj_objF, grasp.grasp_objF)
            wrench_eeF += gravity_eeF
        arm_stability = arm.InsideTorqueLimits(q, wrench_eeF)
        #print('Grasp: {}, Arm: {}'.format(rgrasp_stability, arm_stability))
        return rgrasp_stability and arm_stability

def stability_indirect(arm, grasp, mating, wrench, q=None, gravity=True, checkToolContact=True):
    tcontact_matingF = (grasp.body.link_from_name('tip')).get_link_tform(worldFrame=False)
    tcontact_matingF[0:3, 0:3] = numpy.eye(3)
    
    tcontact_objF = numpy.dot(mating.grasp_objF, tcontact_matingF) 
    rcontact_objF = getContactFrame(grasp.grasp_objF)
    obj_objF = numpy.eye(4)

    gravity_worldF = numpy.array([0, 0, -9.8*grasp.body.get_mass(), 0, 0, 0]) # cog = com 
    worldF_worldF = numpy.eye(4)
    worldF_worldF[0:3, 3] = grasp.body.get_transform()[0:3, 3]
    gravity_objF = wrenchFrameTransformation(gravity_worldF, worldF_worldF, wrench.body.get_transform())

    wrench_tcontactF = wrenchFrameTransformation(wrench.ft_objF, obj_objF, tcontact_objF)

    #wrench_rcontactF = wrenchFrameTransformation(wrench_tcontactF, rcontact_objF, tcontact_objF) #corrected
    wrench_rcontactF = wrenchFrameTransformation(wrench.ft_objF, obj_objF, rcontact_objF)
    if gravity:
        gravity_rcontactF = wrenchFrameTransformation(gravity_objF, obj_objF, rcontact_objF)
        wrench_rcontactF += gravity_rcontactF

    tgrasp_stability = isStableGrasp(wrench_tcontactF, mu=mating.mu, r=mating.r, N=0)
    rgrasp_stability = isStableGrasp(wrench_rcontactF, mu=grasp.mu, r=grasp.r)

    if q is None:
        if checkToolContact:
            #print('Tool: {}, Grasp: {}'.format(tgrasp_stability, rgrasp_stability))
            return tgrasp_stability and rgrasp_stability
        else: 
            #print('(Dont check tool) Grasp: {}'.format(rgrasp_stability))
            return rgrasp_stability
    else:
        wrench_eeF = wrenchFrameTransformation(wrench_rcontactF, grasp.grasp_objF, rcontact_objF)
        if gravity:
            gravity_eeF = wrenchFrameTransformation(gravity_objF, obj_objF, grasp.grasp_objF)
            wrench_eeF += gravity_eeF
        arm_stability = arm.InsideTorqueLimits(q, wrench_eeF)
        if checkToolContact: 
            #print('Tool: {}, Grasp: {}, Arm: {}'.format(tgrasp_stability, rgrasp_stability, arm_stability))
            return all([tgrasp_stability, rgrasp_stability, arm_stability])
        else:
            #print('(dont check tool), Grasp {}, Arm: {}'.format(rgrasp_stability, arm_stability))
            return rgrasp_stability and arm_stability
