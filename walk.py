# -*- coding: utf-8 -*-
"""
Library for creature walk cycles
walk cycles are a series of poses (see Pose) that is intended to move the creatures.

Created May 2017
rewritten Sept 2020

TODO:
    improve makereasonable
    automatic testing
    
@author: niraj
"""

import numpy as np
import os
import pickle
import time

class Walkcycle:
    """represents the walk cycle of a creature. Where its limbs go in order to walk.
    properties:
        numLimbs, number of limbs
        startPose, a pose object for the desired pose at time t
        depth (int), the number of sin/cos terms the walkcycle uses.
            A larger depth allows smoother and more finely controlled movements,
            but takes longer to calculate.
    """
    def __init__(self, numLimbs, p, q, r, depth):
        """ Constructor for a walk cycle. Probably use one of the other functions
        in this file to create a Walkcycle. This constructor is mostly for being called by them.
        numLimbs is the number of limbs the creature has
        p and q are coefficients for sin and cos respectively.
            Both are (numLimbs x 2 x depth) arrays. Column 0 is polar, 1 is azimuthal.
        r is int vector of length=numLimbs. Values -1, 0, or 1 corresponding
            to the azimuthal angle undergoing a full negative rotation,
            no rotation, or a full positive rotation.
        depth (int) is class property
        """
        self.numLimbs = numLimbs
        assert(p.shape == (self.numLimbs, 2))
        self._p = p
        assert(q.shape == (self.numLimbs, 2))
        self._q = q
        self._r = int(r)
        assert(np.all(np.logical_or(self._r == -1, self._r == 0, self._r == 1)))
        self.depth = int(depth)
        assert(depth > 0)
        self.startPose = self.calcDesiredPose(0.)
    
    def calcDesiredPose(self, cyclePlace):
        """return the desired Pose object at cyclePlace.
        cyclePlace (0 to 1) is how far the walk cycle is along its path"""
        cycleAngle = 2*np.pi*cyclePlace
        fourierAngles = cycleAngle*np.arange(1, self.depth+1).reshape(1,1,self.depth)
        desiredPose = np.sum(self._p*np.sin(fourierAngles) + self._q*np.sum(fourierAngles), axis=2)
        desiredPose[:,1] += cycleAngle*self._r
        
    def variation(self, creat, courseChange, fineChange):
        """return a walkcycle that is similar to this one.
        courseChange > 0 is how much the large-scale behaviour
            (low frequency terms) of the walk cycle changes
        fineChange > 0 is how much the small details (high frequency terms)
            of the walk cycle changes
        having either change > 1 is a lot"""
        assert(courseChange > 0)
        assert(fineChange > 0)
        p = self._p
        q = self._q
        # the amount each number can change is a*d+b where d is depth
        a = (fineChange - courseChange)/self.depth
        b = courseChange - a
        
        change = 0.1/self.depth*np.random.random(size=(self.numLimbs,2,self.depth))
        change *= (b*np.ones(self.depth) + a*np.arange(1,self.depth+1)).reshape((1,1,self.depth))
        p += change
        change = 0.1/self.depth*np.random.random(size=(self.numLimbs,2,self.depth))
        change *= (b*np.ones(self.depth) + a*np.arange(1,self.depth+1)).reshape((1,1,self.depth))
        q += change
        if np.random.random() * courseChange > 0.8:
            r = np.random.randint(-1,2)
        cycle = Walkcycle(self.numLimbs, p, q, r, self.depth)
        return cycle
        
            
    def save(self, fileName):
        """ save the Walkcycle to a pickle file
            fileName is a string. File is saved to walkcycles/fileName.walk.
        """
        assert(len(fileName) > 0)
        fileName = os.path.join("walkcycles", fileName + ".walk")
        with open(fileName, 'wb') as outFile:
            pickle.dump(self, outFile, 0)
    

def loadWalkcycle(fileName, newDepth=None):
    """returns a Walkcycle saved by Walkcycle.save()
    newDepth (optional) if you want to change the depth of the walkcycle"""
    fileName = os.path.join("walkcycles", fileName + ".walk")
    try:
        with open(fileName, 'rb') as inFile:
            cycle = pickle.load(inFile)
    except:
        print("Couldn't load", fileName)
        return None
    if not newDepth == cycle.depth:
        if newDepth > cycle.depth:
            p = np.zeros(cycle.numlimbs, 2, newDepth)
            q = np.zeros(cycle.numlimbs, 2, newDepth)
            p[:,:,0:cycle.depth] = cycle._p
            q[:,:,0:cycle.depth] = cycle._q
        else:
            p = cycle._p[:,:,0:newDepth]
            q = cycle._q[:,:,0:newDepth]
        cycle._p = p
        cycle._q = q
        cycle.depth = newDepth
    return cycle

def genSimpleCycle(creat):
    """given a creature, return a simple basic walk cycle object for that creature.
    Deterministic."""
    
def makeReasonable(cycle):
    """ when a new walkcycle is generated, use this fcn to make it reasonable"""
    maxValues = np.zeros((cycle.numLimbs, 2, cycle.order))
    for cyclePlace in np.arange(0,1,0.01):
        isNewMax = 1*(np.abs(cycle.calcDesiredPose(cyclePlace)) > maxValues)
        
    



