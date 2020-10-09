# -*- coding: utf-8 -*-
"""
A class for a creature simulator. Contains Many Creatures and can simulate their physics.

Created May 2020 @author: Niraj

TODO:
    add creature interaction
    add creature thinking and larger-scale state
    multithreading?
    I should maybe replace the parallel lists in Simulator with a class?
"""

import numpy as np
import time
import CreatPhysics as phys
import Pose

h = 1e-3 #timestep for physics update
pupdatetime = 0

class CreaturePhysState:
    """ A class for the physical state of a creature.
    This should be overloaded by a physState for the individual Creature.
    Data:
    pos is an nx3 array of n positions in 3D (x,y,z) space.
    vel is the derivative of pos (same shape).
    rotation is an mx3x3 array of rotation matrices such that
        (nx3 coordinates in global frame) = (nx3 coordinates in body frame) @ RotationMatrix[i,:,:]
    if state is a relative position instead of absolute, it will store rot
    angvel is an mx3 array of m angular velocities in 3D (x,y,z) space.
        they are the derivatives of the rotation matrices.
    """
    
    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self, value):
        self._pos[:] = value
    
    @property
    def vel(self):
        return self._vel
    @vel.setter
    def vel(self, value):
        self._vel[:] = value
        
    @property
    def rotation(self):
        return self._rot
    @rotation.setter
    def rotation(self, value):
        self._rot[:] = value
    
    @property
    def angvel(self):
        return self._angvel
    @angvel.setter
    def angvel(self, value):
        self._angvel[:] = value
    
    def __init__(self, stateToCopy = None, numPositions = 1, numRotations = 0, isDelta = False):
        """ numPositions is n, the number of position vectors
        numRotations is m, the number of rotation matrices
        isDelta = True if this is a relative position, not an absolute
        """
        if stateToCopy is None:
            self._nPos = numPositions
            self._nRot = numRotations
        else:
            self._nPos = stateToCopy._nPos #is this legal
            self._nRot = stateToCopy._nRot
        self._pos = np.zeros((self._nPos, 3))
        self._vel = np.zeros((self._nPos, 3))
        if isDelta:
            self._rot = np.zeros((self._nRot, 3))
        else:
            self._rot = np.repeat(np.identity(3).reshape((1,3,3)), repeats = self._nRot, axis = 0)
        self._angvel = np.zeros((self._nRot, 3))
        self.isDelta = isDelta
    
    def set_from_motion(self, oldState, linAcc, angAcc, div = 1.):
        """given that a creature is moving from oldState
        and has calculated linAcc (linear acceleration as nx3) and
        angAcc (angular acceleration as mx3), set self to the state after movement.
        div is for Runge-Kutta calculations.
        h is simulator.h
        This is designed so oldState can be self and it'll work.
        """
        self.pos = oldState.pos + oldState.vel * h / div
        self.vel = oldState.vel + linAcc * h / div
        self.rotation = oldState.rotation @ phys.angvel_to_rotmat(oldState.angvel * h / div)
        self.angvel = oldState.angvel + angAcc * h / div
    
    def add(self, other):
        """adds a delta state to this state."""
        assert(other.isDelta)
        self._pos += other.pos
        self._vel += other.vel
        self._angvel += other.angvel
        self.rotation = self.rotation @ phys.angvel_to_rotmat(other._rot)
    
    def __itruediv__(self, divisor):
        """divides a delta by a number"""
        self._pos /= divisor
        self._vel /= divisor
        self._rot /= divisor
        self._angvel /= divisor
        return self
        
    def set_to_zero(self):
        self.pos *= 0
        self.vel *= 0
        self.rotation *= 0
        self.angvel *= 0
        
def calc_deltaState(oldState, linAcc, angAcc, deltaState):
    """given that a creature is moving from oldState
    and has calculated linAcc (linear acceleration as nx3) and
    angAcc (angular acceleration as mx3),
    set deltaState for runge-kutta final average step.
    h is simulator.h
    """
    deltaState.pos += oldState.vel * h
    deltaState.vel += linAcc * h
    deltaState.rotation += oldState.angvel * h
    deltaState.angvel += angAcc * h

class Simulator:
    """ A simulator runs a simulation.
    Properties:
    h is the simulation step time
    creatures is the array of creatures to be simulated
    physStates is the states of each creature
    t is the current simulation time
    """
    
    @property
    def h(self):
        return self._h
    
    def __init__(self):
        self.creatures = []
        self.physStates = []
        self._h = 1e-3
        self._integratorStates = []
        self._integratorDeltaStates = []
        self.desiredPoses = []
        self.t = 0.
    
    def addCreature(self, creature):
        """add a creature to the list of creatures in the simulator.  """
        self.creatures.append(creature)
        self.physStates.append(creature.newPhysState())
        self.desiredPoses.append(None)
        self._integratorStates.append(creature.newPhysState())
        self._integratorDeltaStates.append(CreaturePhysState(
                stateToCopy = self._integratorStates[-1], isDelta = True))
    
    def removeCreature(self, index):
        del self.creatures[index]
        del self.physStates[index]
        del self._integratorStates[index]
        del self._integratorDeltaStates[index]
    
    def pUpdate(self):
        """ uses 4th order Runge-Kutta integration method to update the physics"""
        startTime = time.clock()
        for i in range(len(self.creatures)):
            self.creatures[i].logger.t = self.t
            #first, creature chooses the force it wants
            walkForce = Pose.calcWalkForce(self.creatures[i], self.physStates[i],
                                           self.desiredPoses[i])
            ############## LOGGGING
#            if self.desiredPoses[i] is None:
#                desiredPoseCart = self.physStates[i].limbPos
#            else:
#                desiredPoseCart = Pose.poseToPhysState(self.creatures[i],
#                                self.physStates[i], self.desiredPoses[i]).limbPos
#            self.creatures[i].logger["desiredPoseCart"].addPoint(desiredPoseCart)
#            if walkForce is None:
#                self.creatures[i].logger["walkForce"].addPoint(np.zeros((self.creatures[i].numLimbs,3)))
#            else:
#                self.creatures[i].logger["walkForce"].addPoint(walkForce)
            ################## END LOGGING
            #then calculate physics
            self._integratorDeltaStates[i].set_to_zero()
            #k1
            oldState = self.physStates[i]
            deltaState = self._integratorDeltaStates[i]
            linAcc, angAcc = self.creatures[i].getAcc(oldState, walkForce = walkForce)
            calc_deltaState(oldState, linAcc, angAcc, deltaState)
            self._integratorStates[i].set_from_motion(
                    oldState, linAcc, angAcc, div = 1.)
            #k2
            oldState = self._integratorStates[i]
            linAcc, angAcc = self.creatures[i].getAcc(oldState, walkForce = walkForce)
            calc_deltaState(oldState, linAcc, angAcc, deltaState)
            self._integratorStates[i].set_from_motion(
                    oldState, linAcc, angAcc, div = 2.)
            #k3
            linAcc, angAcc = self.creatures[i].getAcc(oldState, walkForce = walkForce)
            calc_deltaState(oldState, linAcc, angAcc, deltaState)
            self._integratorStates[i].set_from_motion(
                    oldState, linAcc, angAcc, div = 2.)
            #k4
            linAcc, angAcc = self.creatures[i].getAcc(oldState, walkForce = walkForce)
            calc_deltaState(oldState, linAcc, angAcc, deltaState)
            
            deltaState /= 6
            
            self.physStates[i].add(deltaState)
        self.t += self.h
        global pupdatetime
        pupdatetime += time.clock() - startTime
