# -*- coding: utf-8 -*-
"""
Library for creature walk cycles
walk cycles are a series of poses (see Pose) that is intended to move the creatures.
use genSimpleCycle to get a new walkcycle
use walkcycle.variation to get a similar walkcycle

Created May 2017
rewritten Sept 2020

TODO:
    better way to find max in makeReasonable
    automatic testing
    make variation all gaussian
    Walkcycle method for changing depth
    
@author: niraj
"""

import numpy as np
import os
import pickle
import time
from CreatPhysics import wrap_to_pi
import evolution #for testing
import logger

class Walkcycle:
    """represents the walk cycle of a creature. Where its limbs go in order to walk.
    properties:
        numLimbs, number of limbs
        startPose, a pose object for the desired pose at time t
        depth (int), the number of sin/cos terms the walkcycle uses.
            A larger depth allows smoother and more finely controlled movements,
            but takes longer to calculate.
    """
    def __init__(self, numLimbs, a0, a, b, r, depth):
        """ Constructor for a walk cycle. Probably use one of the other functions
        in this file to create a Walkcycle. This constructor is mostly for being called by them.
        numLimbs is the number of limbs the creature has
        a and b are coefficients for sin and cos respectively.
            Both are (numLimbs x 2 x depth) arrays. Column 0 is polar, 1 is azimuthal.
        a0 is the average value of polar and azimuthal angle. (numLimbs x 2) array.
        r is int vector of length=numLimbs. Values -1, 0, or 1 corresponding
            to the azimuthal angle undergoing a full negative rotation,
            no rotation, or a full positive rotation.
        depth (int) is class property
        """
        self.numLimbs = numLimbs
        self.depth = int(depth)
        assert(depth > 0)
        assert(a.shape == (self.numLimbs, 2, self.depth))
        self._a = a
        assert(b.shape == (self.numLimbs, 2, self.depth))
        self._b = b
        assert(a0.shape == (self.numLimbs, 2))
        self._a0 = a0
        self._r = r.astype(int)
        assert(np.all(np.logical_or.reduce((self._r == -1, self._r == 0, self._r == 1))))
        _makeReasonable(self)
        self.startPose = self.calcDesiredPose(0.)
    
    def calcDesiredPose(self, cyclePlace):
        """return the desired Pose object at cyclePlace.
        cyclePlace (0 to 1) is how far the walk cycle is along its path"""
        cycleAngle = 2*np.pi*cyclePlace
        fourierAngles = cycleAngle*np.arange(1, self.depth+1).reshape(1,1,self.depth)
        desiredPose = np.sum(  self._a * np.sin(fourierAngles)
                             + self._b * np.cos(fourierAngles), axis=2)
        desiredPose += self._a0
        desiredPose[:,1] += cycleAngle*self._r
        desiredPose[:,0] = np.abs(desiredPose[:,0]) #polar angle is positive
        return desiredPose
        
    def variation(self, courseChange, fineChange):
        """return a walkcycle that is similar to this one.
        courseChange > 0 is how much the large-scale behaviour
            (low frequency terms) of the walk cycle changes
        fineChange > 0 is how much the small details (high frequency terms)
            of the walk cycle changes
        having either change > 1 is a lot"""
        assert(courseChange > 0)
        assert(fineChange > 0)
        a = self._a
        b = self._b
        a0 = self._a0
        # the amount each number can change is m*d+c where d is depth
        #course represents depth 1 most strongly, fine represents depth 10
        m = (fineChange - courseChange)/(10 - 1)
        c = courseChange - m
        
        change = 0.1/self.depth*np.random.random(size=(self.numLimbs,2,self.depth))
        change *= (c*np.ones(self.depth) + m*np.arange(1,self.depth+1)).reshape((1,1,self.depth))
        a += change
        change = 0.1/self.depth*np.random.random(size=(self.numLimbs,2,self.depth))
        change *= (c*np.ones(self.depth) + m*np.arange(1,self.depth+1)).reshape((1,1,self.depth))
        b += change
        
        a0 += 0.1*(np.random.random(size=(self.numLimbs, 2)) - 0.5)
#        if np.random.random() * courseChange > 0.8:
#            r = np.random.randint(-1,2)
#        else:
#            r = self._r
        r = np.where(np.random.random(self.numLimbs) * courseChange > 0.5, 
                     np.random.randint(-1,2, size = self.numLimbs), self._r)
        cycle = Walkcycle(self.numLimbs, a0, a, b, r, self.depth)
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
    if not newDepth is None and not newDepth == cycle.depth:
        if newDepth > cycle.depth:
            a = np.zeros((cycle.numLimbs, 2, newDepth))
            b = np.zeros((cycle.numLimbs, 2, newDepth))
            a[:,:,0:cycle.depth] = cycle._a
            b[:,:,0:cycle.depth] = cycle._b
        else:
            a = cycle._a[:,:,0:newDepth]
            b = cycle._b[:,:,0:newDepth]
        cycle._a = a
        cycle._b = b
        cycle.depth = newDepth
    return cycle

def genSimpleCycle(creat, depth):
    """given a creature, return a simple basic walk cycle object for that creature.
    depth is the integer depth, as described in walkcycle docs
    Deterministic."""
    depth = int(depth)
    a0 = np.ones((creat.numLimbs, 2))
    a0[:,0] = np.arcsin(creat.limbSizes / creat.limbLengths).reshape(creat.numLimbs) #set on ground
    a0[:,1] *= np.pi #start at the top
    a = np.zeros((creat.numLimbs, 2, depth))
    b = np.zeros((creat.numLimbs, 2, depth))
    #one full azimuthal rotation (r = +-1). Direction depends on which side of creature.
    r = (2*np.logical_or(creat.limbSides == 0, creat.limbSides == 1) - 1
                  ).reshape(creat.numLimbs)
    return Walkcycle(creat.numLimbs, a0, a, b, r, depth)

def compareWalkCycles(c1, c2):
    """Returns a number > 0 that represents how similar two walk cycles are to each other.
    Commutative. Absolute number doesn't matter, only comparisons relative to other cycles."""
    #TODO: this 
    
def reverseWalkcycle(cycle):
    """Does your walkcycle go backwards? 
    This fcn returns the same walkcycle but in the opposite direction."""
    #TODO: this

############# internal use functions
def _makeReasonable(cycle):
    """ when a new walkcycle is generated, use this fcn to make it reasonable"""
    maxDisp = np.zeros(cycle.numLimbs)
    NUM_STEPS = 100
    #max polar angle is pi when below horizontal, pi/2 when above
    #TODO: there's probably a better way to find the max than just looping through
    for i in range(NUM_STEPS):
        desiredPose = cycle.calcDesiredPose(i/NUM_STEPS)
        #multiply by 2 if it's above the horizontal, because the polar limit is 2x as small there
        desiredPose[:,0] = np.where(np.abs(wrap_to_pi(desiredPose[:,1])) > np.pi/2,
                                    2*desiredPose[:,0], desiredPose[:,0])
        isNewMax = (desiredPose[:,0] > maxDisp)
        maxDisp = np.where(isNewMax, desiredPose[:,0], maxDisp)
    scale = np.where(maxDisp > np.pi, 3.14/maxDisp, 1) #the 3.14 is < pi so we don't go over
    cycle._a0[:,0] *= scale
    cycle._a[:,0,:] *= scale.reshape(cycle.numLimbs, 1)
    cycle._b[:,0,:] *= scale.reshape(cycle.numLimbs, 1)
    
####################### Testing

def walkcycleTest(loud = False):
    """run a full suite of tests on walkcycles and walkcycle generation"""
    #TODO: use the logger to store time taken. Needs new logger class.
    
    print("Walkcycle Test 1")
    #Test 1: generate a bunch of walkcycles, check their max values and execution time
    numCreatures = 10
    numCycles = 5
    numRunIncrements = 10000
    
    print("Creature | # limbs | depth | creation time (ms) | calc time (ns)")
    for creatNum in range(numCreatures):
        creat = evolution.newRandomCreature()
        depth = random.randint(3, 10)
        cycle = genSimpleCycle(creat, depth)
        createTimes = np.empty(numCycles)
        runTimes = np.empty(numCycles)
        for cNum in range(numCycles):
            startTime = time.process_time()
            cycle = cycle.variation(0.8, 0.5) #i picked those numbers
            createTimes[cNum] = time.process_time() - startTime
            
            poses = np.empty((creat.numLimbs, 2, numRunIncrements))
            startTime = time.process_time()
            for i in range(numRunIncrements):
                poses[:,:,i] = cycle.calcDesiredPose(i/numRunIncrements)
            runTimes[cNum] = time.process_time() - startTime
            limitCheck = np.where(wrap_to_pi(np.abs(poses[:,1,:])) > np.pi / 2,
                                  2 * poses[:,0,:], poses[:,0,:])
            if (np.any(limitCheck > np.pi)):
                print("Cycle", cNum, "of creature", creatNum, "out of range")
                #more info about this? maybe check we aren't close to the corner
        print("{0:8d} | {1:7d} | {2:5d} | {3:7.2f} +- {4:7.2f} | {5:6.3f} +- {6:6.3f}".format(
                creatNum, creat.numLimbs, depth, 
                np.average(createTimes)*1000, np.std(createTimes)*1000,
                np.average(runTimes) * 1e6 / numRunIncrements, 
                np.std(runTimes) * 1e6 / numRunIncrements))
    
    print("Walkcycle Test 2")
    #Test 2: test saving and loading
    for creatNum in range(numCreatures):
        print("Creature", str(creatNum+1)+'/'+str(numCreatures))
        creat = evolution.newRandomCreature()
        depth = random.randint(3, 10)
        cycle = genSimpleCycle(creat, depth)
        for cNum in range(numCycles):
            #print("\tSave/Load test", str(cNum + 1) + ": ", end = '')
            #First test without changing depth
            cycle.save("test")
            testCycle = loadWalkcycle("test")
            status = (
                    testSimilarParams(cycle._a0, testCycle._a0)
                and testSimilarParams(cycle._a, testCycle._a, cycle.depth, testCycle.depth)
                and testSimilarParams(cycle._b, testCycle._b, cycle.depth, testCycle.depth)
                and testSimilarParams(cycle._r, testCycle._r))
            if not status:
                print("FAIL in walkcycle save")
                #say more here?
            else: #if it worked
                #Test with new depth
                testCycle = loadWalkcycle("test", newDepth = random.randint(3, 10))
                status = (
                        testSimilarParams(cycle._a0, testCycle._a0)
                    and testSimilarParams(cycle._a, testCycle._a, cycle.depth, testCycle.depth)
                    and testSimilarParams(cycle._b, testCycle._b, cycle.depth, testCycle.depth)
                    and testSimilarParams(cycle._r, testCycle._r))
                if not status:
                    print("FAIL in walkcycle save depth change (",
                            cycle.depth, testCycle.depth,")")
                    #say more here?
                print("(" + str(cycle.depth) + "," + str(testCycle.depth) + ") ", end = "")
            if status:
                print("Pass ", end="")
        print()
 
def testSimilarParams(p, pCopy, depth = 1, depthCopy = 1):
    if depth == depthCopy:
        return np.allclose(p, pCopy, rtol=0, atol = 1e-12)
    elif depth > depthCopy:
        return np.allclose(p[:,:,0:depthCopy], pCopy, rtol=0, atol = 1e-12)
    elif depth < depthCopy:
        return (np.allclose(pCopy[:,:,depth:], np.zeros(pCopy[:,:,depth:].shape), atol = 1e-12)
                and np.allclose(p, pCopy[:,:,0:depth], rtol=0, atol = 1e-12))
 
if __name__ == "__main__":
    import random
    walkcycleTest(True)
