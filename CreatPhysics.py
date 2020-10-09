# -*- coding: utf-8 -*-
"""
Creatures Physics file for simulating Creatures
a bunch of functions for physics calculations that should work in general

Created Feb 2020 by Niraj

TODO:
    clarify how strength works as a vector
    make sure it doesn't make unnecessary copies
    maybe use masked arrays for faster calculations? Especially in floor force
    pointsRepel is slow for small numbers of repelling points
    get rid of rotationmatrix
    damp leg oscilalations?
    body as impenetrable surfaces
    I don't think angvel_to_rotmat works for multiple rotmats

@author: Niraj
"""

import numpy as np

x_coord = np.array([1.,0.,0.])
y_coord = np.array([0.,1.,0.])
z_coord = np.array([0.,0.,1.])

onlyup_mat_mask = np.array([[0.,0.,0.],[0.,1.,0.],[0.,0.,0.]])
#(nx3 matrix with y = 0) = removeup_mat_mask @ matrix
removeup_mat_mask = np.array([[1.,0.,0.],[0.,0.,0.],[0.,0.,1.]])

removeup_vec_mask = np.array([1.,0.,1.])
#(only y part of array as a size nx1 array) =  matrix @ onlyup_vec_mask
onlyup_vec_mask = np.array([0.,1.,0.]).reshape(3,1)

def unit(v):
    """return the normalized vector in the same direction as v with length = 1"""
    return v / np.linalg.norm(v, axis=1)

def airDrag(vel, strength = 2):
    """ given a points with velocities, find the forces from air drag on those points. 
    vel is a nx3 (x,y,z) array of velocities
    strength is scalar or 1xn strength of the forces.
    returns forces as nx3 """
    force = -strength * np.sqrt(np.sum(vel**2, axis=1, keepdims = True)) * vel
    return force

def interactFloor(pos, size, velocity, force = None, n = None, strength = 6000,
                  kinetic_fric = 0.4, static_fric = 0.6):
    """given a set of points, make them not fall through the floor and get floor friction.
    
    pos is nx3 (x,y,z) points, size is the radius of those points.
    velocity is the (nx3) matrix of velocities of those points
    force is the (nx3) forces on those points.
        if force is None (default), a nx3 matrix of zeros is created.
    n is number of points, by default gotten from size of pos.
    strength is a scalar or 1xn vector of how strong the repulsion is.
    kinetic_ and static_ fric are the coefficients of friction,
        0 <= kinetic_fric <= static_fric <= 1.
    
    returns a nx3 for the total force including the new reaction force from the
            ground and the frictional forces
    Because of the nature of static friction, this step should be calculated last.
    If a point doesn't break static friction, the force on the point will be
        0 in the x/z directions.
        
    This function takes the input force and returns a force that includes the input.
        So do creatForce = interactFloor(..., force = creatForce) rather than creatForce +=
    
    """
    if n == None:
        n = pos.size[1]
    if force is None:
        force = np.zeros((n,3))
    startForce = force #for debugging, delete this
    height = (pos @ onlyup_vec_mask) - size
    belowFloor = (height < 0)
    normalForce = belowFloor * (-height) * strength
    sidewaysVelocity = belowFloor * (velocity @ removeup_mat_mask)
    sidewaysVelMag = np.linalg.norm(sidewaysVelocity, axis = 1, keepdims = True)
    sidewaysVelMag += 1e-8 # get rid of 0s that cause singularity
    sidewaysVelDir = sidewaysVelocity / sidewaysVelMag
    sidewaysForce = force @ removeup_mat_mask
    sidewaysForceMag = np.linalg.norm(sidewaysForce, axis=0)
    #static fric breaks if velocity is above a threshold or force is above a threshold
    staticFricBroken = np.logical_or(1 - belowFloor,
                                     np.linalg.norm(sidewaysVelocity, axis=1, keepdims = True) > 1e-5,
                                     sidewaysForceMag > normalForce*static_fric)
    
    #remove forces cancelled by static friction:
    force = ((force + normalForce) @ onlyup_mat_mask) + (staticFricBroken * sidewaysForce)
    #add kinetic friction
    force += belowFloor * staticFricBroken * normalForce * kinetic_fric * -sidewaysVelDir
    #take out velocities stopped by static friction
    velocity = (velocity @ onlyup_mat_mask) + (staticFricBroken * sidewaysVelocity)
    if np.any((1 - belowFloor) * (force != startForce)):
        print("Above floor but ground forces exerted")
        print("height", np.round(height, 4))
        print("stat fr brok", staticFricBroken)
        print("force", np.round(force,4))
        print("startForce", np.round(startForce, 4))
        print()
    return force
    

def bodyConnection(pos1, pos2, nominaldistances, strength = 10000, errorThresh = 2):
    """ p1 and p2 are groups of n points each in 3D space. 
    This function returns two nx3 matrices:
        - the elastic force on the points p1 by p2. That is the negative of the
          force on p2 by p1.
        - the euclidean distance vectors from p1 to p2
    
    pos1 is the positions of the points p1 as 3 rows (x, y z) and n columns
    pos2 is like pos1 but for p2
    nominaldistances is the distances the points should be apart from each other.
    If they get farther, they'll pull together, and if they get closer, 
        they'll push apart.
    strength is how strongly they affect each other. Can be a scalar value, or
        an array of length n if the different points have different strengths.
    If errorThresh is not None, then this function throws an error if the distance 
        is errorThresh times more than it should be.
    """
    connLength = pos2 - pos1
    connLengthMag = np.linalg.norm(connLength, axis=1, keepdims = True)
    connForce = strength * ((connLengthMag - nominaldistances) * connLength) / connLengthMag
#    forceAngle = np.abs(np.sum((connForce / np.linalg.norm(connForce, axis = 1, keepdims = True))
#                               * (connLength / connLengthMag), axis = 1))
#    if not np.any(np.logical_and(forceAngle > 0.999, forceAngle < 1.001)):
#        print(forceAngle)
                       
#    if (not errorThresh == None) and np.any(connLength > errorThresh*nominaldistances):
#            raise ValueError("connection stretched to beyond threshold")
    return connForce

def pointsRepel(pos, sizes, n=None, strength=6000):
    """ given a set of n objects, find the contact forces between them.
    pos is 3 (x,y,z) by n matrix of point positions.
    sizes is the radius of each object, 1xn.
    strength is how strong the axis is, scalar or 1xn. 
    Returns force nx3.
    Force has magnitude 0 if they are not touching,
    and pushes them apart if they are.
    """
    if n == None:
        n = np.shape(pos)[0]
    force = np.zeros((n, 3))
    timesThrough = np.int(np.floor(n/2))
    for i in range(1,timesThrough+1):
        #starting with 1 because points don't affect themselves, end at n/2 because the other half are equal and opposite
        distance = np.roll(pos,-i, axis = 0) - pos # dist_i = pos_j - pos_i (nx3), distances as 3-vectors
        mag = np.linalg.norm(distance, axis = 1, keepdims = True)
        nominalLength = sizes + np.roll(sizes,-i, axis = 0) #the distance they should be apart, r_i + r_j
        newForce = (mag < nominalLength)*strength*(mag - nominalLength)*distance/mag
        force += newForce
        #the force of i on j is the negative of the force of j on i
        #except if it's the last through on an even n, then it would double count
        if not (i == timesThrough and n % 2 == 0):
            force -= np.roll(newForce, i, axis = 0)
    return force

def angvel_to_rotmat(angvel, m = None):
    """ takes an angular velocity as a mx3 array and converts it to mx3x3 rotation matrices.
    new_rotmat = old_rotmat @ angVel_to_rotMat(angvel)[i,:,:]
    m is the number of angular velocities.
    Uses the Rodriguez formula.
    """
    if m is None:
        m = angvel.shape[0]
    angle = np.linalg.norm(angvel, axis = 1, keepdims = True)
    if np.abs(angle) < 1e-12:
        return np.identity(3)
    axis = angvel / angle
    #so2 is in the mathematical group so(2), representing angvel raised to a skew-sym matrix
    so2 = np.zeros((m, 3, 3))
    so2[:,1,2] = axis[:,0]
    so2[:,2,0] = axis[:,1]
    so2[:,0,1] = axis[:,2]
    so2 -= so2.transpose([0,2,1])
    return np.identity(3) + so2 * np.sin(angle) + so2 @ so2 * (1 - np.cos(angle))
