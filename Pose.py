# -*- coding: utf-8 -*-
"""
File for using creature Poses, which is the configuration the creature "chooses" to position their limbs.

Created Jun 2017, rewritten August 2020
@author: niraj

TODO:
    Should pose be a class?
"""
import numpy as np
import copy
from CreatPhysics import unit, x_coord, y_coord, z_coord


def getPose(creature, physState, includeBodyLimbVecs = False):
    """gets a creature's pose from its physical state.
    For a boxer creature, pose is (numlimbs x 2) array.
    For each limb, the first element is the angle (radians) between the limb and
        "outwards", the vector pointing away from the side the limb is attached to.
    the second element is the angle between the limb and "down" in the plane of
        the side the limb is attached to.
    second element is positive to the creature's left.
    
    Equivalent to polar and azimuthal angle in spherical coordinates with pole outward and reference direction down
    returns pose if includeBodyLimbVecs == False (default)
    if includeBodyLimbVecs == True, returns (pose, bodyLimbVecs), where
        bodyLimbVecs is the vectors from shoulder to foot in the body frame
    """
    pose = np.empty((physState.numLimbs, 2))
    shoulders = creature.getShoulders(physState)
    limbVecs = physState.limbPos - shoulders #vectors in direction of legs
    bodyLimbVecs = limbVecs @ physState.bodyRot.transpose()
    outward = creature.getLimbOutward(physState)
    outwardPart = np.sum(limbVecs * outward, axis=1)
    pose[:,0] = np.arccos(outwardPart / np.linalg.norm(limbVecs, axis=1)) #polar angle
    planarLimbVecs = limbVecs - outwardPart.reshape(creature.numLimbs,1) * outward #limb vectors projected in down-forward plane
    #azimuthal angle
    pose[:,1] = np.arccos(np.tensordot(planarLimbVecs, -y_coord @ physState.bodyRot, axes=([1],[0]))
                          / np.linalg.norm(planarLimbVecs, axis=1)) #angle to down
    pose[:,1] *= (2 * (np.tensordot(planarLimbVecs, x_coord @ physState.bodyRot, axes=([1],[0])) > 0) - 1) #flip sign if the angle towards forward < 0
    if includeBodyLimbVecs:
        return pose, bodyLimbVecs
    else:
        return pose
        
def printPose(creat, pose, digits = 4):
    print("----------- Pose -------------------------")
    with np.printoptions(precision=digits):
        print("Out:", pose[:,0])
        print("Around:", pose[:,1])
        
def calcDesiredPoseForce(creature, bodyLimbVecs, desiredPose):
    """given a creature, its current body limbVecs, and its desired position,
    return desired cartesian force to move it to desiredPose in body coordinates"""
    error = poseLimbVecs(creature, desiredPose) - bodyLimbVecs
    kp = 100.
    return kp*error
    
def constrainWalkForce(creature, walkForce, bodyLimbVecs):
    """constrain a walkForce to what a creature can produce.
    maximum magnitude and doesn't allow leg to stretch and compression
    returns the constrained force in body coordinates"""
    maxForce = 8000.
    radialDirection = bodyLimbVecs / creature.limbLengths
    walkForce -= np.sum(walkForce * radialDirection, axis = 1, keepdims=True) * radialDirection
    magnitude = np.sum(walkForce**2, axis=1, keepdims=True)
    magnitude = np.where(magnitude > maxForce, maxForce, magnitude) #constrain to maxForce
    walkForce = unit(walkForce) * magnitude
    return walkForce

def poseLimbVecs(creature, pose):
    """given a creature and a pose, 
    return the cartesian vectors for its limbs, in the BODY frame.
    (They need to be rotated into global frame) ex. limbVecs = limbVecs @ physState.bodyRot"""
    # be aware this is coordinates for each leg. z is out, x is down.
    # z/r = cos(theta) * zhat
    limbVecs = np.cos(pose[:,0]).reshape(creature.numLimbs,1) * creature.limbOutwardBodyFrame
    # x/r = sin(theta)cos(phi) * xhat
    limbVecs += np.outer(np.sin(pose[:,0])*np.cos(pose[:,1]), -y_coord)
    # y/r = sin(theta)cos(phi) * yhat
    forward = -np.cross(creature.limbOutwardBodyFrame, y_coord)
    limbVecs += (np.sin(pose[:,0])*np.sin(pose[:,1])).reshape(creature.numLimbs,1) * forward
    limbVecs *= creature.limbLengths # multiply by r
    return limbVecs

def poseToPhysState(creature, currPhysState, pose):
    """given a creature, it's current physState, and a desired pose, find the physState
    with the same body position and rotation, but limbs in locations determined by the pose.
    returns the new physState."""
    state = copy.deepcopy(currPhysState)
    limbVecs = poseLimbVecs(creature, pose)
    #this is in the body coordinates of the creature, we need to rotate into global
    limbVecs = limbVecs @ currPhysState.bodyRot
    state.limbPos = creature.getShoulders(currPhysState) + limbVecs
    return state
    
def calcWalkForce(creat, physState, desiredPose):
    """calculate the walkForce, which is the force the creature "decides" to have,
    as an nx3 array. creat is the creature, physState is the creature's current
    physState, desiredPose is the creature's desired pose.
    returns None if desiredPose is None."""
    if desiredPose is None:
        return None
    else:
        currPose, bodyLimbVecs = getPose(creat, physState, includeBodyLimbVecs = True)
        walkForce = calcDesiredPoseForce(creat, bodyLimbVecs, desiredPose)
        walkForce = constrainWalkForce(creat, walkForce, bodyLimbVecs)
        return walkForce @ physState.bodyRot #rotate into global coordinates

def natStartPose(creat):
    """ a natural starting pose that's always stable. returns a pose."""
    pose = np.empty((creat.numLimbs, 2))
    pose[:,0] = np.arcsin(creat.limbSizes / creat.limbLengths).reshape(creat.numLimbs) #limbs rest on ground
    pose[:,1] = np.pi * np.ones(creat.numLimbs) # limbs rest on ground
    return pose

def straightUpPose(creat):
    """Return a pose where all the feet are pointing down. If you want that."""
    pose = np.ones((creat.numLimbs, 2))
    pose[:,0] *= np.pi/2
    pose[:,1] *= 0
    return pose

#########################################################################
# old code from when poses were calculated WRT forward
#def getPose(creature, physState, includeBodyLimbVecs = False):
#    """gets a creature's pose from its physical state.
#    For a boxer creature, pose is (numlimbs x 2) array.
#    For each limb, the first element is the angle (radians) between the limb and
#        "outwards", the vector pointing away from the side the limb is attached to.
#    the second element is the angle between the limb and "down" in the plane of
#        the side the limb is attached to.
#    second element is positive to the creature's left.
#    
#    Equivalent to polar and azimuthal angle in spherical coordinates with pole outward and reference direction down
#    returns pose if includeBodyLimbVecs == False (default)
#    if includeBodyLimbVecs == True, returns (pose, bodyLimbVecs), where
#        bodyLimbVecs is the vectors from shoulder to foot in the body frame
#    """
#    pose = np.empty((physState.numLimbs, 2))
#    shoulders = creature.getShoulders(physState)
#    limbVecs = physState.limbPos - shoulders #vectors in direction of legs
#    bodyLimbVecs = limbVecs @ physState.bodyRot.transpose()
#    forward = x_coord @ physState.bodyRot #this is from when it was judged by creature's forward
#    forwardPart = np.tensordot(limbVecs, forward, axes=([1],[0]))
#    pose[:,0] = np.arccos(forwardPart / np.linalg.norm(limbVecs, axis=1)) #polar angle
#    planarLimbVecs = limbVecs - np.outer(forwardPart, forward) #limb vectors projected in down-outwards plane
#    #azimuthal angle
#    pose[:,1] = np.arccos(np.tensordot(planarLimbVecs, -y_coord @ physState.bodyRot, axes=([1],[0]))
#                          / np.linalg.norm(planarLimbVecs, axis=1)) #angle to down
#    pose[:,1] *= (2 * (np.tensordot(planarLimbVecs, x_coord @ physState.bodyRot, axes=([1],[0])) > 0) - 1) #flip sign if the angle towards forward < 0
#    if includeBodyLimbVecs:
#        return pose, bodyLimbVecs
#    else:
#        return pose

#def calcDesiredPoseAcc(creature, currPose, desiredPose):
#    """given a creature, its current position, and its desired position,
#    return desired acceleration in pose coordinates"""
#    error = desiredPose - currPose
#    #azimuthal needs to wrap to pi
#    error[:,1] = np.where(error[:,1] >  np.pi, 2*np.pi - error[:,1], error[:,1])
#    error[:,1] = np.where(error[:,1] < -np.pi, 2*np.pi + error[:,1], error[:,1])
#    
#    kp = 200.
#    print("current")
#    printPose(creature, currPose)
#    print("poseAcc")
#    printPose(creature, kp*error)
#    return kp*error

#def limitPoseAcc(creature, pose_acceleration):
#    """limit a pose_acceleration to what a creature can produce.
#    returns the limited pose_acceleration"""
#    #max radial accelerations
#    maxPolar = 40.*creature.limbLengths.reshape(creature.numLimbs)
#    maxAzimuthal = 40.*creature.limbLengths.reshape(creature.numLimbs)
#    #constrain polar acceleration
#    pose_acceleration[:,0] = np.minimum(pose_acceleration[:,0], maxPolar)
#    pose_acceleration[:,0] = np.maximum(pose_acceleration[:,0], -maxPolar)
#    #constrain azimuthal acceleration
#    pose_acceleration[:,1] = np.minimum(pose_acceleration[:,1], maxAzimuthal)
#    pose_acceleration[:,1] = np.maximum(pose_acceleration[:,1], -maxAzimuthal)
#    return pose_acceleration

#def forcePoseToCart(creature, physState, currPose, pose_acceleration):
#    """given desired acceleration in the polar and azimuthal directions as
#    described by Pose, return the desired 3D force in cartesian space.
#    creature is the creature, physState and currPose are the current physState and pose of the creature,
#    pose_acceleration is (numlimbs x 2), a polar and azimuthal angle for each limb.
#    returns (numlimbs x 3) 3D force for each limb."""
#    """we have the direction and magnitude in the azimuthal and polar directions,
#    we want to convert to cartesian, so we use relationship between azimuthal 
#    and polar directions and cartesian directions. """
#    """with xhat meaning the x direction, theta is polar angle and phi is azimuthal angle:
#    d(position) = dr*rhat + r*dtheta*thetahat + r*sin(theta)*dphi*phihat
#    we ignore change in r because limbs can't move in that direction (stretch/compress)
#    d(position) = r*dtheta*thetahat + r*sin(theta)*dphi*phihat
#    dx = <d(position), xhat> """
#    """note that we aren't using the usual convention of z-axis being the pole (sorry)
#    we're using x axis is the pole, azimuthal angle = 0 at the negative y axis.
#    yhat = -thetahat*cos(theta)*cos(phi) + phihat*sin(phi)        (conventionally -xhat)
#    zhat = -thetahat*cos(theta)*sin(phi) - phihat*cos(phi)        (conventionally -yhat)
#    xhat = -thetahat*sin(theta)                                   (conventionally zhat)"""
#    cartForce = np.empty((creature.numLimbs, 3)) #force in cartesian coordinates
#    #deltax/r = -deltatheta*sin(theta)
#    cartForce[:,0] = -pose_acceleration[:,0]*np.sin(currPose[:,0])
#    #deltay/r = -deltatheta*cos(theta)*cos(phi) + sin(theta)*deltaphi*sin(phi)
#    cartForce[:,1] = (-pose_acceleration[:,0]*np.cos(currPose[:,0])*np.cos(currPose[:,1])
#                      +np.sin(currPose[:,0])*pose_acceleration[:,1]*np.sin(currPose[:,1]))
#    #deltaz/r = -deltatheta*cos(theta)*sin(phi) - sin(theta)*deltaphi*cos(phi)
#    cartForce[:,2] = (-pose_acceleration[:,0]*np.cos(currPose[:,0])*np.sin(currPose[:,1])
#                      -np.sin(currPose[:,0])*pose_acceleration[:,1]*np.cos(currPose[:,1]))
#    cartForce *= creature.limbLengths #multiply by r
#    #this is in the body coordinates of the creature, we need to rotate into global
#    cartForce = cartForce @ physState.bodyRot
#    return cartForce

#def forcePoseToCart(creature, physState, currPose, pose_acceleration):
#    """given desired acceleration in the polar and azimuthal directions as
#    described by Pose, return the desired 3D force in cartesian space.
#    creature is the creature, physState and currPose are the current physState and pose of the creature,
#    pose_acceleration is (numlimbs x 2), a polar and azimuthal angle for each limb.
#    returns (numlimbs x 3) 3D force for each limb."""
#    """we have the direction and magnitude in the azimuthal and polar directions,
#    we want to convert to cartesian, so we use relationship between polar and
#    cartesian coordinates.
#    With xhat meaning the x direction, theta is polar angle and phi is azimuthal angle:
#    d(position) = dr*rhat + r*dtheta*thetahat + r*sin(theta)*dphi*phihat
#    we ignore change in r because limbs can't move in that direction (stretch/compress)
#    d(position) = r*dtheta*thetahat + r*sin(theta)*dphi*phihat
#    dx = <d(position), xhat>
#    The z-axis (pole) is outwards, y-axis is forward, x-axis is down. Unfortunately,
#        that's different from the world coordinate frame, where y is down.
#    xhat = thetahat*cos(theta)*cos(phi) - phihat*sin(phi)
#    yhat = thetahat*cos(theta)*sin(phi) + phihat*cos(phi)
#    zhat = -thetahat*sin(theta)                          """
#    forward = -np.cross(creature.limbOutwardBodyFrame, y_coord)
#    ####cartForce = np.empty((creature.numLimbs, 3)) #force in cartesian coordinates
#    #deltaz/r = -deltatheta * sin(theta) * zhat
#    cartForce = -(pose_acceleration[:,0] * np.sin(currPose[:,0])).reshape(creature.numLimbs,1) * creature.limbOutwardBodyFrame
#    #deltax/r = (deltatheta*cos(theta)*cos(phi) - sin(theta)*deltaphi*sin(phi)) * xhat
#    cartForce += np.outer((pose_acceleration[:,0]*np.cos(currPose[:,0])*np.cos(currPose[:,1])
#                           - np.sin(currPose[:,0])*pose_acceleration[:,1]*np.sin(currPose[:,1]))
#                          , -y_coord)
#    #deltay/r = (deltatheta*cos(theta)*sin(phi) + sin(theta)*deltaphi*cos(phi)) * yhat
#    cartForce += ((pose_acceleration[:,0]*np.cos(currPose[:,0])*np.sin(currPose[:,1])
#                   + np.sin(currPose[:,0])*pose_acceleration[:,1]*np.cos(currPose[:,1])).reshape(creature.numLimbs, 1)
#                  * forward)
#    cartForce *= creature.limbLengths #multiply by r
#    #this is in the body coordinates of the creature, we need to rotate into global
#    cartForce = cartForce @ physState.bodyRot
#    return cartForce


#def poseToPhysState(creature, currPhysState, pose):
#    """given a creature, it's current physState, and a desired pose, find the physState
#    with the same body position and rotation, but limbs in locations determined by the pose.
#    returns the new physState."""
#    state = copy.deepcopy(currPhysState)
#    limbVecs = np.empty((creature.numLimbs, 3))
#    """note that we aren't using the usual convention of z-axis being the pole (sorry)
#    we're using x axis is the pole, azimuthal angle = 0 at the negative y axis"""
#    # x/r = cos(theta)       (conventionally z)
#    limbVecs[:,0] = np.cos(pose[:,0])
#    # y/r = -sin(theta)cos(phi)       (conventionally -x)
#    limbVecs[:,1] = -np.sin(pose[:,0])*np.cos(pose[:,1])
#    # z/r = -sin(theta)cos(phi)       (conventionally -y)
#    limbVecs[:,2] = -np.sin(pose[:,0])*np.sin(pose[:,1])
#    limbVecs *= creature.limbLengths # multiply by r
#    
#    #this is in the body coordinates of the creature, we need to rotate into global
#    limbVecs = limbVecs @ currPhysState.bodyRot
#    state.limbPos = creature.getShoulders(currPhysState) + limbVecs
#    return state
    

#def natStartPose(creat):
#    """ a natural starting pose that's always stable. returns a pose."""
#    pose = np.zeros((creat.numLimbs, 2))
#    pose[:,0] += np.pi/2 * np.logical_or(creat.limbSides == 1, creat.limbSides == 3).reshape(creat.numLimbs) #outwards
#    pose[:,0] += (np.arcsin(creat.limbSizes / creat.limbLengths)
#                  * (creat.limbSides == 0)).reshape(creat.numLimbs) #limbs rest on ground
#    pose[:,0] += ((np.pi - np.arcsin(creat.limbSizes / creat.limbLengths))
#                   * (creat.limbSides == 2)).reshape(creat.numLimbs) #limbs rest on ground
#    
#    pose[:,1] += np.pi * np.logical_or(creat.limbSides == 0, creat.limbSides == 2).reshape(creat.numLimbs) # limbs rest on ground
#    pose[:,1] -= ((np.pi/2 + np.arcsin(creat.limbSizes / creat.limbLengths))
#                   * (creat.limbSides == 1)).reshape(creat.numLimbs) #limbs rest on ground
#    pose[:,1] += ((np.pi/2 + np.arcsin(creat.limbSizes / creat.limbLengths))
#                   * (creat.limbSides == 3)).reshape(creat.numLimbs) #limbs rest on ground
#    return pose
    