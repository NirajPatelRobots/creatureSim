# -*- coding: utf-8 -*-
"""
Library for saving, loading, and displaying the actions of creatures
Created Jan 2017 @author: niraj

TODO:
    associate runs with creatures somehow
    make the saved state more like the physState object
    safer file opening and closing
"""

import numpy as np
import os.path
import pyximport
pyximport.install(language_level = 3)

csavepath = os.path.dirname(__file__) + os.sep + "creatures"#"C:/Users/niraj/Documents/programs/creatureSim/creatures"

def newFile(fileName = None, askForFileName = False):
    """creates a file for saving with savePlace.
    askForFileName is True if the system should prompt for filename.
    fileName is the name of the file"""
    if askForFileName:
        fileName = input("Enter filename for movement data save: ")+".txt"
    elif fileName is None:
        fileName = "recent.txt"
    if fileName == ".txt":
        fileName = "recent.txt"  # the default
    if fileName == "exit.txt":
        return (None, None)
    fileName = "animations/" + fileName
    inFile = open(fileName, 'w')
    inFile.write("#pos (x y z) axis (x y z) up (x y z) limb positions (x y z)\n")
    return inFile, fileName

def savePlace(inFile, physState):
    for i in range(3):
        inFile.write(str(physState.bodyPos[i])+' ')
    for i in range(3):
        inFile.write(str(physState.bodyRot[0, i]) + ' ')
    for i in range(3):
        inFile.write(str(physState.bodyRot[1, i]) + ' ')
    for limbNum in range(physState.numLimbs):
        for i in range(3):
            inFile.write(str(physState.limbPos[limbNum, i]) + ' ') 
    inFile.write('\n')

def saveCreature(creature, fileName = None): #saves a creature to a file
    if fileName is None:
        fileName = input("Enter filename for creature save: ")
    if fileName == "exit":
        return
    elif fileName == "":
        fileName = "recent.creat"
    if not fileName[-6:] == ".creat":
        fileName += ".creat"
    inFile = open(os.path.join(csavepath,fileName), 'w')
    inFile.write(creature.printable())
    inFile.close()

def loadCreature(fileName):
    import Creature 
    if fileName == "exit":
        return
    if not fileName[-6:] == ".creat":
        fileName += ".creat"
    try:
        inFile = open(os.path.join(csavepath,fileName), 'r')
    except:
        print("Invalid filename: "+fileName)
        return
    newlimbs = []
    for i, line in enumerate(inFile):
        if i==1:
            split = line.split()
            newbodySize = np.array([float(split[2]),float(split[4]),float(split[6])])
        elif i>=3:
            split = line.split()
            newlimbs.append(Creature.Limb(side = int(split[1]), location = float(split[3]),
                                          length = float(split[5]), size = float(split[7])))
    inFile.close()
    return Creature.Creature(bodySize = newbodySize, limbs = newlimbs, name = fileName.split('.')[0])

    