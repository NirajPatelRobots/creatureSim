# -*- coding: utf-8 -*-
"""
Library for information about "Boxer" creatures
information about their body structure, how their physics work, etc

Created Jan 2017 (lots of code created earlier)
rewritten continuously until march 2020
@author: niraj

TODO:
    faster physics (cython?)
    make it possible to have other types of creature
    body has friction against the ground (should be easy)
    uneven ground
    resolve the difference between list of Limbs in the Creature and array storage
 """

import numpy as np
import time
import pyximport
pyximport.install(language_level = 3)
import CreatPhysics as phys
import simulator
import logger
import Pose


x_coord = np.array([1.,0.,0.])
y_coord = np.array([0.,1.,0.])
z_coord = np.array([0.,0.,1.])
onlyup = np.array([0,1,0])  #masks for np
removeup = np.array([1,0,1])
alternating1 = np.array([-1, -1, -1, -1, 1, 1, 1, 1])
alternating2 = np.array([-1, -1, 1, 1, 1, 1, -1, -1])
alternating3 = np.array([-1, 1, -1, 1, -1, 1, -1, 1])
GRAVITY_STRENGTH = 6 #how strong gravity is, in m/s^2

getacctime = 0
limbtime = 0

class Limb:
    def __init__(self, side, location, length, size, num = 0):
        self.side = int(side) # int 0-3 indicates limb is on front, right, back, left side of the creature
        self.location = float(location) # float 0-1 how far down the side
        self.length = float(length)  #how long the limb is
        self.size = float(size)  #how big the end of the limb is
        self.num = num
        #physics
        self.mass = 10*(4/3.0*np.pi*self.size**3)

class Creature:
    def __init__(self, bodySize, limbs, name = None):
        if name == None:
            self.name = ""
        else:
            self.name = name
        self.bodySize = [float(bodySize[0]), float(bodySize[1]), float(bodySize[2])] #list of [length, width, height]
        self.sideLengths = [bodySize[1],bodySize[0],bodySize[1],bodySize[0]]
        self.limbs = sorted(limbs, key= lambda l: l.side*10+l.location)  #list of limb objects organized by side then location
        self.numLimbs = len(limbs)
        for i in range(self.numLimbs):
            self.limbs[i].num = i
        
        #physics
        self.mass = 2*self.bodySize[0]*self.bodySize[1]*self.bodySize[2]
        momI = np.diag([self.bodySize[1]**2+self.bodySize[2]**2,
                        self.bodySize[0]**2+self.bodySize[1]**2,
                        self.bodySize[0]**2+self.bodySize[2]**2])
        self.momI_inv = np.linalg.inv(momI)
        self.limbPos = np.zeros((self.numLimbs, 3))
        self.limbVel = np.zeros((self.numLimbs, 3))
        self.limbSides = np.zeros((self.numLimbs, 1))
        self.limblocs = np.zeros((self.numLimbs, 1))
        self.limbSizes = np.zeros((self.numLimbs, 1))
        self.limbMasses = np.zeros((self.numLimbs, 1))
        self.limbLengths = np.zeros((self.numLimbs, 1))
        self.limbOutwardBodyFrame = np.zeros((self.numLimbs, 3))
        self.v = None #this is for visual animation
        #get corners in body frame
        #array 3 rows (x, y, z), 8 columns (8 corners)
        self.cornersBodyFrame = (np.outer(alternating1, x_coord*self.bodySize[0]/2)
                   + np.outer(alternating2, y_coord*self.bodySize[2]/2)
                   + np.outer(alternating3, z_coord*self.bodySize[1]/2))
        
        ################### limbs
        for limb in self.limbs:
            self.limbSides[limb.num, 0] = limb.side
            self.limblocs[limb.num, 0] = limb.location
            self.limbSizes[limb.num, 0] = limb.size
            self.limbMasses[limb.num, 0] = limb.mass
            self.limbLengths[limb.num, 0] = limb.length
            if limb.side == 0:
                self.limbOutwardBodyFrame[limb.num, :] = x_coord
            elif limb.side == 1:
                self.limbOutwardBodyFrame[limb.num, :] = z_coord
            elif limb.side == 2:
                self.limbOutwardBodyFrame[limb.num, :] = -x_coord
            elif limb.side == 3:
                self.limbOutwardBodyFrame[limb.num, :] = -z_coord
            
        ###################### shoulder positions of each limb relative to body
        #array of 3 rows (x, y, z), one column for each limb
        self.shouldersBodyFrame = np.zeros((self.numLimbs, 3))
        self.shouldersBodyFrame -= self.bodySize[2]*y_coord/2
        self.shouldersBodyFrame += np.outer(((.5*(self.limbSides == 0) - .5*(self.limbSides == 2)) +
                                             ((1*(self.limbSides == 3) - 1*(self.limbSides == 1)) * 
                                              (self.limblocs - 0.5))),
                                            (self.bodySize[0] * x_coord))
        self.shouldersBodyFrame += np.outer(((.5*(self.limbSides == 1) - .5*(self.limbSides == 3)) +
                                             ((1*(self.limbSides == 0) - 1*(self.limbSides == 2)) * 
                                              (self.limblocs - 0.5))),
                                            (self.bodySize[1] * z_coord))
        
        #   Logging
        self.logger = logger.Logger(self.name)
        self.logger.newArrayLog("up", shape = (1, 3))
        self.logger.newArrayLog("limbPos", shape = (self.numLimbs, 3))
        self.logger.newArrayLog("shoulders", shape = self.shouldersBodyFrame.shape)
        self.logger.newArrayLog("corners", shape = (8, 3))
        self.logger.newArrayLog("bodyVel", shape = (1, 3))
        self.logger.newArrayLog("angVel", shape = (1, 3))
        self.logger.newArrayLog("angAcc", shape = (1, 3))
        self.logger.newArrayLog("force", shape = (1,3))
        self.logger.newArrayLog("cornersForce", shape = (8, 3))
        self.logger.newArrayLog("walkForce", shape = (self.numLimbs, 3))
        self.logger.newArrayLog("bodyConnectForce", shape = (self.numLimbs, 3))
        self.logger.newArrayLog("pointsRepel", shape = (self.numLimbs, 3))
        self.logger.newArrayLog("limbForce", shape = (self.numLimbs, 3)) 
        self.logger.newArrayLog("floorLimbForce", shape = (self.numLimbs, 3))
        self.logger.newArrayLog("limbAirDrag", shape = (self.numLimbs, 3))
        self.logger.newArrayLog("torque", shape = (1,3))
        self.logger.newArrayLog("bodyLegTorque", shape = (self.numLimbs,3))
        self.logger.newArrayLog("cornersTorque", shape = (8, 3))
        self.logger.newArrayLog("bodyLegTouchForce", shape = (self.numLimbs, 3))
        self.logger.newArrayLog("desiredPoseCart", shape = (self.numLimbs, 3))
        
    def printable(self):
        text = ("Creature "+ hex(id(self)) + (' "'+self.name+'"\n' if self.name else "\n") +
                "Body size "+ str(round(self.bodySize[0], 3))+' x '+str(round(self.bodySize[1], 3))+' x '+
                str(round(self.bodySize[2], 3))+"\n"+str(self.numLimbs)+" limbs:\n")
        for limb in self.limbs:
            text += ("\tSide: "+str(limb.side)+" location: "+str(round(limb.location, 3))+" length: "
                     +str(round(limb.length, 3))+" size: "+str(round(limb.size, 3))+'\n')
        return text
    
    def newPhysState(self, pose = None):
        """ Creates a new physical state object for this creature.
        The default (None) is the creature with all limbs pointing straight down.
        The creature is almost touching the ground.
        """
        physState = BoxerPhysState(creature = self)
        if pose is None:
            physState.limbPos = self.getShoulders(physState) - np.outer(self.limbLengths, y_coord)
        else:
            physState = Pose.poseToPhysState(self, physState, pose)
        lowest = 0.
        for i in range(self.numLimbs):
            if physState.limbPos[i,1] - self.limbSizes[i] < lowest: #bottom of limb
                lowest = physState.limbPos[i,1] - self.limbSizes[i]
        corners = self.getCorners(physState)
        for i in range(8):
            if corners[i,1] < lowest: #bottom of limb
                lowest = corners[i,1]
        startPos = (lowest - 0.1) * y_coord
        physState.pos -= startPos
        return physState
    
    def getCorners(self, physState):
        """ find positions of the 8 corners """
        return np.matmul(self.cornersBodyFrame, physState.bodyRot) + physState.bodyPos
    
    def getShoulders(self, physState):
        """ find shoulders' positions """
        return np.matmul(self.shouldersBodyFrame, physState.bodyRot) + physState.bodyPos
    
    def getLimbOutward(self, physState):
        """ find unit vectors pointing outward for each limb """
        return np.matmul(self.limbOutwardBodyFrame, physState.bodyRot)
        
    def getAcc(self, physState, walkForce = None):
        """ return 2 matrices:
            - linear acceleration
            - angular acceleration
        Each as a 2D matrix with 3 columns for (x, y, z)
        
        walkForce is force on each limb for walking
        """
        startTime = time.clock()
        if walkForce is None:
            walkForce = np.zeros((self.numLimbs, 3))
        bodyPos = physState.bodyPos
        limbPos = physState.limbPos
        bodyVel = physState.bodyVel
        limbVel = physState.limbVel
        shoulders = self.getShoulders(physState)
#        self.logger["shoulders"].addPoint(shoulders)
#        self.logger["limbPos"].addPoint(limbPos)
#        self.logger["bodyVel"].addPoint(bodyVel)
        force = -GRAVITY_STRENGTH * self.mass * y_coord
        
        limbStartTime = time.clock()
        global limbtime
        limbtime += (time.clock() - limbStartTime)
        limbForce = -GRAVITY_STRENGTH * np.outer(self.limbMasses, y_coord)
        
        bodyConnectForce = phys.bodyConnection(limbPos, shoulders, self.limbLengths)
#        self.logger["bodyConnectForce"].addPoint(walkForce + bodyConnectForce)
        limbForce += walkForce + bodyConnectForce
        
        pointsRepelForce = phys.pointsRepel(limbPos, self.limbSizes, self.numLimbs)
        limbForce += pointsRepelForce
        
#        limbForce += self.logger["limbAirDrag"].addPoint(phys.airDrag(limbVel))
        limbForce += phys.airDrag(limbVel)
        
        up = physState.bodyRot[1, :]
#        self.logger["up"].addPoint(up.reshape((1,3)))
        limbVecs = limbPos - shoulders
        upness = limbVecs @ up.reshape((3,1)) + self.limbSizes
        inwardDirection = -self.getLimbOutward(physState)
        inness = np.sum(limbVecs * inwardDirection, axis = 1, keepdims = True) + self.limbSizes
        bodyLegTouchForce = ((upness > 0) * (inness > 0) * -6000
                             *((upness >= inness) * inness * inwardDirection
                               + (upness < inness) * upness * up))
#        self.logger["bodyLegTouchForce"].addPoint(bodyLegTouchForce)
        limbForce += bodyLegTouchForce
        force -= np.sum(walkForce + bodyConnectForce + bodyLegTouchForce, axis=0)
        bodyLegTorque = np.cross((limbPos-bodyPos),
                                 -(walkForce + bodyConnectForce + bodyLegTouchForce))
#        self.logger["bodyLegTorque"].addPoint(bodyLegTorque)
        torque = np.sum(bodyLegTorque, axis = 0)
#        prevLimbForce = limbForce.copy()
        limbForce = phys.interactFloor(limbPos, self.limbSizes, limbVel,
                                       force = limbForce, n = self.numLimbs)
#        self.logger["floorLimbForce"].addPoint(limbForce - prevLimbForce)
        
        ############################### body interact with ground #######################
        corners = self.getCorners(physState)
#        self.logger["corners"].addPoint(corners)
        cornersForce = np.zeros((8,3))
        for c in range(8): #8 corners
            if corners[c][1] < 0: #corner touching ground
                cornersForce[c,1] = -6000*corners[c,1]
#        self.logger["cornersForce"].addPoint(cornersForce)
        force += np.sum(cornersForce, axis = 0)
        
        torque += np.sum(np.cross(corners - bodyPos, cornersForce), axis = 0)
        
        dragForce = -3*np.linalg.norm(bodyVel)*bodyVel
        force += dragForce
        
        linAcc = np.empty((self.numLimbs+1, 3))
        linAcc[0, :] = force/self.mass
        linAcc[1:self.numLimbs+1, :] = limbForce / self.limbMasses
                
        angAcc = self.momI_inv @ torque
#        self.logger["force"].addPoint(force.reshape((1,3)))
#        self.logger["torque"].addPoint(torque.reshape((1,3)))
#        self.logger["angAcc"].addPoint(angAcc.reshape((1,3)))
        
        global getacctime
        getacctime += time.clock() - startTime
        return linAcc, angAcc
    
class BoxerPhysState(simulator.CreaturePhysState):
    """this subclass of the creature physical state is specifically for the
    boxer creatures. 
    """
    @property
    def bodyPos(self):
        return self._pos[0,:]
    @bodyPos.setter
    def bodyPos(self, value):
        self._pos[0,:] = value
    
    @property
    def limbPos(self):
        return self._pos[1:self.numLimbs + 1,:]
    @limbPos.setter
    def limbPos(self, value):
        self._pos[1:self.numLimbs + 1,:] = value
        
    @property
    def bodyVel(self):
        return self._vel[0,:]
    @bodyVel.setter
    def bodyVel(self, value):
        self._vel[0,:] = value
    
    @property
    def limbVel(self):
        return self._vel[1:self.numLimbs + 1,:]
    @limbVel.setter
    def limbVel(self, value):
        self._vel[1:self.numLimbs + 1,:] = value
    
    @property
    def bodyRot(self):
        return self._rot[0, :, :]
    @bodyRot.setter
    def bodyRot(self, value):
        self._rot[0, :, :] = value
    
    @property
    def bodyAngVel(self):
        return self._angvel[0, :]
    @bodyAngVel.setter
    def bodyAngVel(self, value):
        self._angvel[0, :, :] = value
    
    def __init__(self, creature = None):
        """Creature is the specific creature to create this for
        """
        if creature is None:
            self.numLimbs = 0
        else:
            self.numLimbs = creature.numLimbs
        simulator.CreaturePhysState.__init__(self, numPositions = 1 + self.numLimbs, numRotations = 1)
            
            