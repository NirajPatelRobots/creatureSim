# -*- coding: utf-8 -*-
"""
File for Brain class and other functions related to creature decision making.

TODO:
    Check inputs for freshness when start thinking?
    Wrapper for brain to set its own outputs instead of mc using self.brain._output?
    check correct type in setInput?
    Currently, if inputs are added in the getAcc calculations that happen 4 times
        every timestep, inputs will only save the first value. Is that okay?
    motioncontrol needs a method to check if it's close enough to the desiredPose
Created Nov 8 2020

@author: Niraj
"""

import numpy as np
from collections import defaultdict
import Pose

class Brain:
    """Brains make decisions for creatures.
    Brains have inputs (senses) and outputs (decisions).
    brain.think() is called by the simulator to get the brain to produce all outputs.
    
    To set/get inputs/outputs, use brain.setInput() and brain.getOutput().
    Inputs:
        physState - the physState of the creature
    Outputs:
        walkForce - the force the creature wants to exert on its legs. None for no movement.
    """
    def __init__(self, creature, timestep):
        """Constructor.
        creature is the creature this brain is for.
        timestep is the simulation timestep h."""
        self.creat = creature
        self._h = timestep
        # inputs
        self._input = defaultdict()
        self._inputTime = defaultdict()
        # outputs
        self._output = {}
        # sections
        self.mc = MotionControl(self)
        
        self.reset()
        
    def getOutput(self, outputName):
        """outputName is which output you want."""
        return self._output[outputName]
    
    def think(self):
        """This method makes all the decisions."""
        #TODO: an input checker?
        self.mc.think()
    
    def setInput(self, inputName, value, alwaysSet = False):
        """set a certain input. Doesn't change if it's already been set this timestep
        inputName is the name of the input (string),
        value is the value to set it to,
        alwaysSet is True if you've already checked needsInput(inputName, simTime)
            and don't need to check again"""
        if alwaysSet or self.needsInput(inputName):
            #TODO: check type is valid?
            self._input[inputName] = value
            self._inputTime[inputName] = self.simTime
        
    def needsInput(self, inputName):
        """returns boolean, True if an input has NOT already been set this timeset."""
        return self._inputTime[inputName] < self.simTime
        
    def reset(self):
        """resets to a neutral state"""
        # input
        self.simTime = -1. #so these are invalid and get overwritten later
        self.setInput("physState", None, alwaysSet = True)
        self.simTime = 0.
        # sections
        self.mc.reset()


class MotionControl:
    """This class is the part of the Brain that chooses motion.
    mc.status for setting and getting status. Can be "relax", "stand", "walk".
    The motion controller is a state machine W.R.T. status.
    It is a member variable of a Brain class. It also has a member variable for its brain.
    """
    def __init__(self, brain):
        """constructor"""
        import walk
        self.brain = brain
        #statuses
        self.state = {}
        self.walkCycle = walk.genSimpleCycle(self.brain.creat, 3)
        #output
        self.brain._output["walkForce"] = None
    
    def reset(self):
        """used to reset to a neutral state"""
        self._status = "relax"
        self.state["walkSpeed"] = .5
        self.state["walkCyclePlace"] = 0.
        self.desiredPose = None
    
    def think(self):
        """The main method that makes it do its thing"""
        if not self.status == "relax":
            if self.desiredPose is None: #if you're not relaxing, you need a place to go
                self.status = "relax"
                
        if self.status == "walk":
            self.desiredPose = self.walkCycle.calcDesiredPose(self.state["walkCyclePlace"])
            self.state["walkCyclePlace"] += self.brain._h * self.state["walkSpeed"]
            self.state["walkCyclePlace"] %= 1.0
                
        if self.desiredPose is None:
            self.brain._output["walkForce"] = None
        else:
            self.brain._output["walkForce"] = Pose.calcWalkForce(self.brain.creat,
                       self.brain._input["physState"], self.desiredPose)
        ############## LOGGING
        if self.desiredPose is None:
            desiredPoseCart = np.zeros((self.brain.creat.numLimbs,3))
            self.brain.creat.logger["walkForce"].addPoint(np.zeros((self.brain.creat.numLimbs,3)))
            print("no desired pose")
        else:
            desiredPoseCart = Pose.poseToPhysState(self.brain.creat,
                            self.brain._input["physState"], self.desiredPose).limbPos
            self.brain.creat.logger["walkForce"].addPoint(self.brain._output["walkForce"])
        self.brain.creat.logger["desiredPoseCart"].addPoint(desiredPoseCart)
        ################## END LOGGING
    
    @property
    def status(self):
        return self._status
    @status.setter
    def status(self, value):
        if value in ["relax", "stand", "walk"]:
            self._status = value
            if value == "relax":
                self.desiredPose = None
            if value == "walk":
                self.desiredPose = self.walkCycle.calcDesiredPose(self.state["walkCyclePlace"])
    
    def setDesiredPose(self, desiredPose):
        """if you want to manually make it go to a Pose"""
        self.desiredPose = desiredPose
        self.status = "stand"
        
        