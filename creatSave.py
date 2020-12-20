# -*- coding: utf-8 -*-
"""
Library for saving, loading, and displaying the actions of creatures
Created Jan 2017 @author: niraj

TODO:
    associate runs with creatures somehow
    make the saved state more like the physState object
    safer file opening and closing
    binary instead of ascii?
"""

import numpy as np
import os.path
import pyximport
pyximport.install(language_level = 3)

csavepath = os.path.dirname(__file__) + os.sep + "creatures"#"C:/Users/niraj/Documents/programs/creatureSim/creatures"

def loadSimDataFile(fileName = None):
    """ returns a file for saving with savePlace. Default is 'recent'.
    Creates if it doesn't exist, appends otherwise.
    fileName is the name of the file"""
    if fileName is None or fileName == "":
        fileName = "recent"
    fileName = "animations/" + fileName + ".txt"
    outFile = open(fileName, 'a')
    outFile.write("#pos (x y z) axis (x y z) up (x y z) limb positions (x y z)...\n")
    return outFile

def savePlace(outFile, physState):
    """saves a physState to a file opened by loadSimDataFile"""
    for i in range(3):
        outFile.write(str(physState.bodyPos[i])+' ')
    for i in range(3):
        outFile.write(str(physState.bodyRot[0, i]) + ' ')
    for i in range(3):
        outFile.write(str(physState.bodyRot[1, i]) + ' ')
    for limbNum in range(physState.numLimbs):
        for i in range(3):
            outFile.write(str(physState.limbPos[limbNum, i]) + ' ') 
    outFile.write('\n')

def deleteSimDataFile(fileName = None):
    if fileName is None or fileName == "":
        fileName = "recent"
    fileName = "animations/" + fileName + ".txt"
    if os.path.exists(fileName):
        os.remove(fileName)

def saveCreature(creature, fileName = None): #saves a creature to a file
    if fileName is None:
        fileName = input("Enter filename for creature save: ")
    if fileName == "exit":
        return
    elif fileName == "":
        fileName = "recent.creat"
    if not fileName[-6:] == ".creat":
        fileName += ".creat"
    outFile = open(os.path.join(csavepath,fileName), 'w')
    outFile.write(creature.printable())
    outFile.close()

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

    