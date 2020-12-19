# -*- coding: utf-8 -*-
"""
A class for a creature simulator. Contains Many Creatures and can simulate their physics.

Created May 2020 @author: Niraj

TODO:
    add creature interaction
    multithreading
    Can save filenames differently
    I should maybe replace the parallel lists (creature, brain, physState)
        in Simulator with a class?
"""

import numpy as np
import time
import CreatPhysics as phys
import creatSave

pupdatetime = 0
h = 1e-3

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
    
    def set_from_motion(self, oldState, linAcc, angAcc, h):
        """given that a creature is moving from oldState
        and has calculated linAcc (linear acceleration as nx3) and
        angAcc (angular acceleration as mx3), set self to the state after movement.
        h is step size, simulator.h
        This is designed so oldState can be the same thing as self and it'll work.
        """
        self.pos = oldState.pos + oldState.vel * h
        self.vel = oldState.vel + linAcc * h
        self.rotation = oldState.rotation @ phys.angvel_to_rotmat(oldState.angvel * h)
        self.angvel = oldState.angvel + angAcc * h
    
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
        
def calc_deltaState(oldState, linAcc, angAcc, deltaState, h):
    """given that a creature is moving from oldState
    and has calculated linAcc (linear acceleration as nx3) and
    angAcc (angular acceleration as mx3),
    set deltaState for runge-kutta final average step.
    h is timestep, simulator.h
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
        self.brains = []
        self.numCreatures = 0
        self.reset()
    
    def addCreature(self, creature):
        """add a creature to the list of creatures in the simulator.  """
        import Think
        self.creatures.append(creature)
        self.physStates.append(creature.newPhysState())
        self.brains.append(Think.Brain(creature, self._h))
        self._integratorStates.append(creature.newPhysState())
        self._integratorDeltaStates.append(CreaturePhysState(
                stateToCopy = self._integratorStates[-1], isDelta = True))
        self.numCreatures += 1
    
    def removeCreature(self, index):
        if index < self.numCreatures:
            del self.creatures[index]
            del self.physStates[index]
            del self.brains[index]
            del self._integratorStates[index]
            del self._integratorDeltaStates[index]
            self.numCreatures -= 1
        else:
            print("WARNING: Tried to remove a creature that doesn't exist")
    
    def run(self, runtime = 5.0, savePlaces = True, printProgress = True):
        """runs the simulation.
        runtime is in seconds, savePlaces is to save creature locations,
        printProgres is true for printing a progress bar."""
        if savePlaces:
            inFile = creatSave.newFile()[0] #technically could leak but ¯\_(ツ)_/¯
        startTime = time.time()
        countmax = int(runtime/self._h)
        for counter in range(countmax):
            self._pUpdate()
            if savePlaces:
                creatSave.savePlace(inFile, self.physStates[0])
            if printProgress and int(counter % (countmax/10)) == 0:
                print("\r\t"+ str(int((float(counter)/countmax)*100)) +"%", end = '') #FANCY PERCENTS
        if savePlaces:
            inFile.close()
            self.creatures[0].logger.save(loud = False)
        if printProgress:
            print("\rTook", round(time.time() - startTime, 3), "seconds to run",
                  countmax * self._h, "seconds of simulation.")
    
    def reset(self):
        """resets creature brains and loggers, but doesn't reset positions."""
        self.t = 0.0
        for i in range(self.numCreatures):
            self.brains[i].reset()
            self.creatures[i].logger.clearAll()
    
    ################## internal use methods
    
    def _pUpdate(self):
        """ uses 4th order Runge-Kutta integration method to update the physics"""
        startTime = time.process_time()
        for i in range(self.numCreatures):
            self.creatures[i].logger.t = self.t
            #add brain inputs
            self.brains[i].setInput("physState", self.physStates[i])
            
            self.brains[i].think()
            self.brains[i].simTime = self.t
            
            #then calculate physics
            walkForce = self.brains[i].getOutput("walkForce")
            self._integratorDeltaStates[i].set_to_zero()
            #k1
            oldState = self.physStates[i]
            deltaState = self._integratorDeltaStates[i]
            linAcc, angAcc = self.creatures[i].getAcc(oldState, walkForce = walkForce)
            calc_deltaState(oldState, linAcc, angAcc, deltaState, self._h)
            self._integratorStates[i].set_from_motion(
                    oldState, linAcc, angAcc, self._h)
            #k2
            oldState = self._integratorStates[i]
            linAcc, angAcc = self.creatures[i].getAcc(oldState, walkForce = walkForce)
            calc_deltaState(oldState, linAcc, angAcc, deltaState, self._h)
            self._integratorStates[i].set_from_motion(
                    oldState, linAcc, angAcc, self._h / 2)
            #k3
            linAcc, angAcc = self.creatures[i].getAcc(oldState, walkForce = walkForce)
            calc_deltaState(oldState, linAcc, angAcc, deltaState, self._h)
            self._integratorStates[i].set_from_motion(
                    oldState, linAcc, angAcc, self._h / 2)
            #k4
            linAcc, angAcc = self.creatures[i].getAcc(oldState, walkForce = walkForce)
            calc_deltaState(oldState, linAcc, angAcc, deltaState, self._h)
            
            deltaState /= 6
            
            self.physStates[i].add(deltaState)
        self.t += self.h
        global pupdatetime
        pupdatetime += time.process_time() - startTime
