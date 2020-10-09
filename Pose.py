# -*- coding: utf-8 -*-
"""
File for using creature Poses, which is the configuration the creature "chooses" to position their limbs.

Created Jun 2017, rewritten August 2020
@author: niraj

TODO:
    Should pose be a class?
    calcDesiredPoseAcc needs work. It gets there eventually, but it's not the shortest distance.
"""
import numpy as np
import random
import copy
from CreatPhysics import unit, x_coord, y_coord, z_coord

pi = np.pi

def getPose(creature, physState):
    """gets a creature's pose from its physical state.
    For a boxer creature, pose is (numlimbs x 2) array.
    For each limb, the first element is the angle (radians) to the creature's forward,
    the second element is the angle between the limb and "down" in the creatures's down-outwards plane.
    second element is positive to the creature's left.
    
    Equivalent to polar and azimuthal angle in spherical coordinates with pole forward and reference direction down
    """
    pose = np.empty((physState.numLimbs, 2))
    shoulders = creature.getShoulders(physState)
    limbVecs = physState.limbPos - shoulders #vectors in direction of legs
    forward = x_coord @ physState.bodyRot
    forwardPart = np.tensordot(limbVecs, forward, axes=([1],[0]))
    pose[:,0] = np.arccos(forwardPart / np.linalg.norm(limbVecs, axis=1)) #polar angle
    planarLimbVecs = limbVecs - np.outer(forwardPart, forward) #limb vectors projected in down-outwards plane
    #azimuthal angle
    pose[:,1] = np.arccos(np.tensordot(planarLimbVecs, -y_coord @ physState.bodyRot, axes=([1],[0])) / np.linalg.norm(planarLimbVecs, axis=1)) #angle to down
    pose[:,1] *= (2 * (np.tensordot(planarLimbVecs, -z_coord @ physState.bodyRot, axes=([1],[0])) > 0) - 1) #flip sign if the angle towards left < 0
    return pose
        
def printPose(creat, pose, digits = 4):
    print("----------- Pose -------------------------")
    print("Forward:", pose[:,0])
    print("Around:", pose[:,1])
    
def calcDesiredPoseAcc(creature, currPose, desiredPose):
    """given a creature, its current position, and its desired position,
    return desired acceleration in pose coordinates"""
    error = desiredPose - currPose
    error = np.where(error > pi, 2*pi-error, error)
    error = np.where(error < -pi, 2*pi+error, error)
    
    kp = 200.
    return kp*error

def limitPoseAcc(creature, pose_acceleration):
    """limit a pose_acceleration to what a creature can produce.
    returns the limited pose_acceleration"""
    #max radial accelerations
    maxPolar = 50.*creature.limbLengths.reshape(creature.numLimbs)
    maxAzimuthal = 50.*creature.limbLengths.reshape(creature.numLimbs)
    #constrain polar acceleration
    pose_acceleration[:,0] = np.minimum(pose_acceleration[:,0], maxPolar)
    pose_acceleration[:,0] = np.maximum(pose_acceleration[:,0], -maxPolar)
    #constrain azimuthal acceleration
    pose_acceleration[:,1] = np.minimum(pose_acceleration[:,1], maxAzimuthal)
    pose_acceleration[:,1] = np.maximum(pose_acceleration[:,1], -maxAzimuthal)
    return pose_acceleration
    
def forcePoseToCart(creature, physState, currPose, pose_acceleration):
    """given desired acceleration in the polar and azimuthal directions as
    described by Pose, return the desired 3D force in cartesian space.
    creature is the creature, physState and currPose are the current physState and pose of the creature,
    pose_acceleration is (numlimbs x 2), a polar and azimuthal angle for each limb.
    returns (numlimbs x 3) 3D force for each limb."""
    """we have the direction and magnitude in the azimuthal and polar directions,
    we want to convert to cartesian, so we use relationship between azimuthal 
    and polar directions and cartesian directions. """
    """with xhat meaning the x direction, theta is polar angle and phi is azimuthal angle:
    d(position) = dr*rhat + r*dtheta*thetahat + r*sin(theta)*dphi*phihat
    we ignore change in r because limbs can't move in that direction (stretch/compress)
    d(position) = r*dtheta*thetahat + r*sin(theta)*dphi*phihat
    dx = <d(position), xhat> """
    """note that we aren't using the usual convention of z-axis being the pole (sorry)
    we're using x axis is the pole, azimuthal angle = 0 at the negative y axis.
    yhat = -thetahat*cos(theta)*cos(phi) + phihat*sin(phi)        (conventionally -xhat)
    zhat = -thetahat*cos(theta)*sin(phi) - phihat*cos(phi)        (conventionally -yhat)
    xhat = -thetahat*sin(theta)                                   (conventionally zhat)"""
    cartForce = np.empty((creature.numLimbs, 3)) #force in cartesian coordinates
    #deltax/r = -deltatheta*sin(theta)
    cartForce[:,0] = -pose_acceleration[:,0]*np.sin(currPose[:,0])
    #deltay/r = -deltatheta*cos(theta)*cos(phi) + sin(theta)*deltaphi*sin(phi)
    cartForce[:,1] = (-pose_acceleration[:,0]*np.cos(currPose[:,0])*np.cos(currPose[:,1])
                      +np.sin(currPose[:,0])*pose_acceleration[:,1]*np.sin(currPose[:,1]))
    #deltaz/r = -deltatheta*cos(theta)*sin(phi) - sin(theta)*deltaphi*cos(phi)
    cartForce[:,2] = (-pose_acceleration[:,0]*np.cos(currPose[:,0])*np.sin(currPose[:,1])
                      -np.sin(currPose[:,0])*pose_acceleration[:,1]*np.cos(currPose[:,1]))
    cartForce *= creature.limbLengths #multiply by r
    #this is in the body coordinates of the creature, we need to rotate into global
    cartForce = cartForce @ physState.bodyRot
    return cartForce
        
def generateRandomPos(numLimbs):
    #TODO: redo this
    #axis
    pos = np.zeros(2*numLimbs+6)
    #print np.shape(pos), "posshape"
    vec1 = np.array([random.random(),random.random(),random.random()])
    mag1 = np.sqrt(np.sum(vec1**2))
    #print np.shape(vec), np.shape(mag), "should be 3 1"
    pos[0:3] = vec1/mag1
    #up
    vec2 = np.array([random.random(),random.random(),0])
    vec2[2] = -(vec1[0]*vec2[0]+vec1[1]*vec2[1])/vec1[2] #make sure it's orthogonal to axis
    mag2 = np.sqrt(np.sum(vec2**2))
    #print np.shape(vec), np.shape(mag), "should be 3 1", mag
    pos[3:6] = vec2/mag2
    #angles
    for i in range(numLimbs):
        pos[6+2*i] = random.uniform(-.5, 1.)*np.pi  #theta
        pos[7+2*i] = random.uniform(0., 1.)*np.pi   #phi
    return pos

def poseToPhysState(creature, currPhysState, pose):
    """given a creature, it's current physState, and a desired pose, find the physState
    with the same body position and rotation, but limbs in locations determined by the pose.
    returns the new physState."""
    state = copy.deepcopy(currPhysState)
    limbVecs = np.empty((creature.numLimbs, 3))
    """note that we aren't using the usual convention of z-axis being the pole (sorry)
    we're using x axis is the pole, azimuthal angle = 0 at the negative y axis"""
    # x/r = cos(theta)       (conventionally z)
    limbVecs[:,0] = np.cos(pose[:,0])
    # y/r = -sin(theta)cos(phi)       (conventionally -x)
    limbVecs[:,1] = -np.sin(pose[:,0])*np.cos(pose[:,1])
    # z/r = -sin(theta)cos(phi)       (conventionally -y)
    limbVecs[:,2] = -np.sin(pose[:,0])*np.sin(pose[:,1])
    limbVecs *= creature.limbLengths # multiply by r
    
    #this is in the body coordinates of the creature, we need to rotate into global
    limbVecs = limbVecs @ currPhysState.bodyRot
    state.limbPos = creature.getShoulders(currPhysState) + limbVecs
    return state
    
def calcWalkForce(creat, physState, desiredPose):
    """calculate the walkForce, which is the force the creature "decides" to have,
    as an nx3 array. creat is the creature, physState is the creature's current
    physState, desiredPose is the creature's desired pose."""
    if desiredPose is None:
        return None
    else:
        currPose = getPose(creat, physState)
        poseAcc = calcDesiredPoseAcc(creat, currPose, desiredPose)
        poseAcc = limitPoseAcc(creat, poseAcc)
        return forcePoseToCart(creat, physState, currPose, poseAcc)
             
def natStartPose(creat):
    """ a natural starting pose that's always stable. returns a pose."""
    pose = np.zeros((creat.numLimbs, 2))
    pose[:,0] += np.pi/2 * np.logical_or(creat.limbSides == 1, creat.limbSides == 3).reshape(creat.numLimbs) #outwards
    pose[:,0] += (np.arcsin(creat.limbSizes / creat.limbLengths)
                  * (creat.limbSides == 0)).reshape(creat.numLimbs) #limbs rest on ground
    pose[:,0] += ((np.pi - np.arcsin(creat.limbSizes / creat.limbLengths))
                   * (creat.limbSides == 2)).reshape(creat.numLimbs) #limbs rest on ground
    
    pose[:,1] += np.pi * np.logical_or(creat.limbSides == 0, creat.limbSides == 2).reshape(creat.numLimbs) # limbs rest on ground
    pose[:,1] -= ((np.pi/2 + np.arcsin(creat.limbSizes / creat.limbLengths))
                   * (creat.limbSides == 1)).reshape(creat.numLimbs) #limbs rest on ground
    pose[:,1] += ((np.pi/2 + np.arcsin(creat.limbSizes / creat.limbLengths))
                   * (creat.limbSides == 3)).reshape(creat.numLimbs) #limbs rest on ground
    return pose

def straightUpPose(creat):
    """Return a pose where all the feet are pointing down. If you want that."""
    pose = np.ones((creat.numLimbs, 2))
    pose[:,0] *= np.pi/2
    pose[:,1] *= 0
    return pose
