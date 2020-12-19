# -*- coding: utf-8 -*-
"""
main program for running things related to creatures
use new, load, save, print, mate to create and manage creatures
drop, stand, walk to run simulations
animate to view simulations
Created on Sat Jan 28 12:34:26 2017

TODO:
    not sure what file it's in, but it makes simplecreature twice
    be better about safely opening files
    runtime slows down after multiple runs???
@author: niraj
"""


import vpython as v
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import importlib
import pyximport
pyximport.install(language_level = 3)

import simulator
import Creature
import creatSave
importlib.reload(creatSave)
import evolution as ev
importlib.reload(ev)
#import walk
#importlib.reload(walk)
import Pose
importlib.reload(Pose)
import animate
importlib.reload(animate)

actCreat = ev.simpleCreature  #start with the simple one
sim = simulator.Simulator()
sim.addCreature(actCreat)

while True:
    args = input(">>>Enter new, save, load, print, mate, drop, stand, walk, animate, or exit: ").split()
    command = args[0] if len(args) > 0 else "" #command is first, the rest of the args are available if needed
    if command == "":
        continue
    command = command.strip().lower()
    if command == "exit":
        sys.exit(0)
    elif command == "code":
        while not command == "exit":
            command = input(">>>")
            try:
                print(eval(command))
            except Exception as e:
                print(e)
    elif command == "new":
        actCreat = ev.newRandomCreature()
        sim.removeCreature(0)
        sim.addCreature(actCreat)
        creatSave.saveCreature(actCreat, fileName = "lastcreat.txt")
        print(actCreat.printable())
    elif command == "print":
        print(actCreat.printable())
    elif command == "save":
        creatSave.saveCreature(actCreat)
    elif command == "load":
        if len(args) > 1:
            retVal = creatSave.loadCreature(args[1])
        else:
            retVal = creatSave.loadCreature(input("Enter saved creature file name: "))
        if not retVal == None:
            actCreat = retVal
            sim.removeCreature(0)
            sim.addCreature(actCreat)
    elif command == "drop":
        sim.physStates[0] = actCreat.newPhysState()
        sim.reset()
        if len(args) > 1:  # first arg is number of seconds
            runtime = float(args[1])
        else:
            runtime = 5.0
        simulator.pupdatetime = 0
        Creature.getacctime = 0
        Creature.limbtime = 0
        sim.brains[0].mc.status = "relax" #go limp
        
        sim.run(runtime)
        print(f"physupdate time: {round(simulator.pupdatetime, 3)} s "
              + f"getacc time: {round(Creature.getacctime, 3)} s "
              + f"limb time: {round(Creature.limbtime, 3)} s "
              #+ (f"({round(Creature.limbtime/actCreat.numLimbs, 2)} s per limb) " 
              # if actCreat.numLimbs > 0 else "")
              )
    elif command == "stand":
        sim.physStates[0] = actCreat.newPhysState(pose = Pose.natStartPose(actCreat))
        sim.reset()
        if len(args) > 1:  # first arg is number of seconds
            runtime = float(args[1])
        else:
            runtime = 5.0
        sim.brains[0].mc.setDesiredPose(Pose.straightUpPose(actCreat))
        #sim.brains[0].mc.setDesiredPose(Pose.natStartPose(actCreat)) #want to sit
        sim.run(runtime)
        Pose.printPose(Pose.getPose(actCreat, sim.physStates[0]))
    elif command == "walk":
        sim.reset()
        sim.brains[0].mc.status = "walk"
        sim.physStates[0] = actCreat.newPhysState(pose = Pose.natStartPose(actCreat))
        # First, settle into the starting pose
        sim.brains[0].mc.state["walkSpeed"] = 0
        sim.run(1.0)
        # then walk
        sim.brains[0].mc.state["walkSpeed"] = 0.5
        sim.run(6.0)
            
    elif command.split()[0] == "animate":
        if len(command.split()) == 2:
            speed = int(command.split()[1])
        else:
            speed = None
        animate.animate_main(actCreat, speed)
    elif command == "mate":
        p1name = input("Enter parent 1 name: ")
        p1 = creatSave.loadCreature(p1name)
        p2name = input("Enter parent 2 name: ")
        p2 = creatSave.loadCreature(p2name)
        actCreat = ev.mate(p1, p2)
        print(actCreat.printable())
    else:
        print("Invalid command")
    
    
    
    