# -*- coding: utf-8 -*-
"""
main program for running things related to creatures
use new, load, save, print, mate to create and manage creatures
drop, stand to run simulations
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
    args = input(">>>Enter new, save, load, print, mate, drop, stand, animate, or exit: ").split()
    command = args[0] if len(args) > 0 else "" #command is first, the rest of the args are available if needed
    if command == "":
        continue
    command = command.strip().lower()
    if command == "exit":
        sys.exit(0)
    elif command == "code":
        while not command == "exit":
            eval(input(">>>"))
    elif command == "new":
        actCreat = ev.makeReasonable(ev.newRandomCreature())
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
        with creatSave.newFile()[0] as inFile:
            sim.physStates[0] = actCreat.newPhysState()
            if len(args) > 1:  # first arg is number of seconds
                countmax = int(float(args[1])/simulator.h)
            else:
                countmax = int(10/simulator.h)
            simulator.pupdatetime = 0
            Creature.getacctime = 0
            Creature.limbtime = 0
            sim.desiredPoses[0] = desiredPose = None #go limp
            startTime = time.time()
            print("Running...")
            counter = 0
            while counter < countmax:
                actCreat.logger.t = counter * simulator.h
                sim.pUpdate()
                creatSave.savePlace(inFile, sim.physStates[0])
                if int(counter % (countmax/10)) == 0:
                    print("\r\t"+ str(int((float(counter)/countmax)*100)) +"%", end = '') #FANCY PERCENTS
                counter += 1
            print("\rTook", round(time.time() - startTime, 3), "seconds to run",
                  countmax * simulator.h, "seconds of simulation.")
            actCreat.logger.save(loud = False)
            Pose.printPose(actCreat, Pose.getPose(actCreat, sim.physStates[0]))
            print(f"physupdate time: {round(simulator.pupdatetime, 3)} s "
                  + f"getacc time: {round(Creature.getacctime, 3)} s "
                  + f"limb time: {round(Creature.limbtime, 3)} s "
                  #+ (f"({round(Creature.limbtime/actCreat.numLimbs, 2)} s per limb) " 
                  # if actCreat.numLimbs > 0 else "")
                  )
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
    elif command == "stand":
        with creatSave.newFile()[0] as inFile:
            #set actCreat physState to the natStartPose
            sim.physStates[0] = actCreat.newPhysState(pose = Pose.natStartPose(actCreat))
            if len(args) > 1:  # first arg is number of seconds
                countmax = int(float(args[1])/simulator.h)
            else:
                countmax = int(5/simulator.h)
            sim.desiredPoses[0] = Pose.straightUpPose(actCreat) #want to stand up straight
            #sim.desiredPoses[0] = Pose.natStartPose(actCreat) #want to sit
            startTime = time.time()
            print("Running...")
            counter = 0
            while counter < countmax:
                actCreat.logger.t = counter * simulator.h
                sim.pUpdate()
                creatSave.savePlace(inFile, sim.physStates[0])
                if int(counter % (countmax/10)) == 0:
                    print("\r\t"+ str(int((float(counter)/countmax)*100)) +"%", end = '') #FANCY PERCENTS
                counter += 1
            print("\rTook", round(time.time() - startTime, 3), "seconds to run",
                  countmax * simulator.h, "seconds of simulation.")
            actCreat.logger.save(loud = False)
            Pose.printPose(actCreat, Pose.getPose(actCreat, sim.physStates[0]))
    elif command == "cycle":
        cycle = walk.genSimpleCycle(actCreat.numLimbs)
        length = np.shape(cycle)[1]
        toplot = np.linalg.norm(cycle, axis=2)
        t = np.linspace(0, length*simulator.h, num = length)
        plt.cla()
        for i in range(actCreat.numLimbs):
            plt.plot(t, toplot[i,:])
        inFile, fileName = creatSave.newFile()
        actCreat.reset(pos=Pos.natStartPos(actCreat))
        success, distance, time = walk.doSimpleCycle(actCreat, Pos.natStartPos(actCreat), cycle, length, inFile)
        if success:
            print("Went", distance, "m in", time, "s.")
        else:
            print("Cycle failed")
            Pos.printPos(actCreat, Pos.natStartPos(actCreat))
            Pos.printPos(actCreat, Pos.getPos(actCreat))
        inFile.close()
    else:
        print("Invalid command")
    
    
    
    