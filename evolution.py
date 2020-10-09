# -*- coding: utf-8 -*-
"""
Evolution for creatures
Creates new creatures from scratch or based on existing creatures.
Created on Sat Dec 24 19:06:28 2016

TODO:
    better way of telling if two limbs should be combined in makeReasonable, and delete smaller one
    easier to change parameters
    
@author: niraj
"""

import random
import numpy as np
import importlib
import pyximport
pyximport.install(language_level = 3)
import Creature
import creatSave #saving and displaying feature
importlib.reload(creatSave)

def geneticLimb(leg1, leg2=None):
    if leg2 == None: #if one leg
        side = leg1.side
        location = leg1.location
        length = leg1.length
        size = leg1.size
    else: #if 2 legs
        side = leg1.side #should be the same
        location = (leg1.location+leg2.location) * .5
        length = (leg1.length+leg2.length) * .5
        size = (leg1.size+leg2.size) * .5
    #randomness
    location *= (random.random()*.4 + .8)
    length *= (random.random()*.4+.8)
    size *= (random.random()*.4+.8)
    return Creature.Limb(side, location, length, size)
    
def newRandomLimb(side = None, location = None, length = None, size = None):
    if side == None:
        side = random.randint(0,3)
    if location == None:
        location = random.random()
    if length == None:
        length = random.random()*3
    if size == None:
        size = random.random()/2
    return Creature.Limb(side, location, length, size)

def mate(parent1, parent2):
    print("Creating child")
    ###########################################  body size  #######################################
    bodySize = [0,0,0]
    for i in range(3):
        bodySize[i] = (parent1.bodySize[i]+parent2.bodySize[i])*0.5 * (random.random()*.4 +.8)
        #each dimension is average of parents * random value 0.8-1.2
    
    ##########################################  number of limbs  ##################################
    limbs = []
    p1limbNum = 0
    p2limbNum = 0
    p1_limbsOnSide = np.array([0,0,0,0])
    p2_limbsOnSide = np.array([0,0,0,0])
    for i in range(p1.numLimbs):
        p1_limbsOnSide[parent1.limbs[i].side] += 1
    for i in range(p2.numLimbs):
        p2_limbsOnSide[parent2.limbs[i].side] += 1
    for sideNum in range(4): #through the 4 sides
        p1limbs = p1_limbsOnSide[sideNum]
        p2limbs = p2_limbsOnSide[sideNum]
        limbNum = 0
        while True:
            if limbNum == p1limbs-1 and limbNum == p2limbs-1:
                #print "both parents have",limbNum+1, "as the last limb on side", sideNum,
                if random.random() > .9:
                    #print "deleting one"
                    break #if parents have the same number of limbs on this side, chance that child has 1 less.
                else:
                    #print "copying that one"
                    #create limb based on the final limb of the side for both parents
                    limbs.append(geneticLimb(parent1.limbs[p1limbNum], parent2.limbs[p2limbNum]))
                    p1limbNum += 1
                    p2limbNum += 1
            elif limbNum < p1limbs and limbNum < p2limbs: #if both parents have this limb
                #make a limb based on both parents' limb
                limbs.append(geneticLimb(parent1.limbs[p1limbNum], parent2.limbs[p2limbNum]))
                p1limbNum += 1
                p2limbNum += 1
                #print "both parents have a limb on side", sideNum
            elif limbNum < p1limbs and limbNum >= p2limbs: #if p1 only has this limb
                #print "parent 1 has a limb on side", sideNum
                if random.random() < 7*parent1.limbs[p1limbNum].size/parent1.sideLengths[sideNum]:
                    #chance to make a limb based on p1's limb that p2 doesn't have
                    #print "\tcreating it"
                    limbs.append(geneticLimb(parent1.limbs[p1limbNum]))
                p1limbNum += 1
            elif limbNum >= p1limbs and limbNum < p2limbs: #if p2 only has this limb
                #print "parent 2 has a limb on side", sideNum
                if random.random() < 7*parent2.limbs[p2limbNum].size/parent2.sideLengths[sideNum]:
                    #chance to make a limb based on p2's limb that p1 doesn't have
                    #print "\tcreating it"
                    limbs.append(geneticLimb(parent2.limbs[p2limbNum]))
                p2limbNum += 1
            elif limbNum == p1limbs and limbNum == p2limbs:
                #print "both parents have",limbNum, "as the last limb on side", sideNum, "may create new"
                if random.random() >= .5:
                    #print "\tcreating new one"
                    # if parents have same # of limbs, chance of child having one extra (chance of 1 less earlier)
                    limbs.append(newRandomLimb(side=sideNum))
            elif limbNum >=p1limbs and limbNum >= p2limbs:
                break
            limbNum += 1
            #print "limbNum, len(limbs):", limbNum, len(limbs)
    return Creature.Creature(bodySize, limbs)

def makeReasonable(creature):
    ####################################   make it more reasonable   ##################################
    limbs = creature.limbs[:]
    bodySize = creature.bodySize[:]
    sideLengths = creature.sideLengths[:]
    if creature.numLimbs == 0:
        return Creature.Creature(bodySize, limbs)
    reasonable = False
    #print "smoothing"
    
    while not reasonable:
        reasonable = True #set it true and if anything changes, it becomes false
        for limb in limbs: #reasonably sized feet
            if limb.length < 2*limb.size:
                limb.length = 2*limb.size
                reasonable = False
        limbs = sorted(limbs, key= lambda l: l.side*10+l.location)
        for limb in limbs: #don't let them be too small, that's unstable
            if limb.size < 0.2:
                limb.size = 0.2
        for limb in limbs: #check for limbs bigger than their side or their length ####################
            oldSize = limb.size
            limb.size = min([limb.size,sideLengths[limb.side]])
            if not oldSize == limb.size:
                #print "limb made smaller because of side length, was", oldSize,'now',limb.size
                reasonable = False
        numberOfLimbs = len(limbs)
        limbNum = 0
        while limbNum < numberOfLimbs - 1: #check for limbs very close, combine them  #######################
            if (limbs[limbNum+1].side == limbs[limbNum].side and
                abs(limbs[limbNum].location-limbs[limbNum+1].location) < 0.1):
                #if on the same side as the next limb and they're very close together, delete next limb
                #and make this one bigger
                limbs[limbNum].size += limbs[limbNum+1].size
                limbs.remove(limbs[limbNum+1])
                numberOfLimbs -= 1
                reasonable = False
                #print "deleting close limbs on side", limbs[limbNum].side
            limbNum += 1
        for i, limb in enumerate(limbs): # check for intersecting (not v close) limbs, make them smaller
            if not i == len(limbs)-1: # if this isn't the last one
                if limbs[i+1].side == limb.side + 1 or limbs[i+1].side == limb.side: #if on same or next side
                    if limbs[i+1].side == limb.side: #if on same side
                        distance = np.sqrt(((limbs[i+1].location-limb.location)*sideLengths[limb.side])**2
                                           + (limb.length - limbs[i+1].length)**2)
                    else: # if on next side, sqrt(x^2 + y^2 + z^2)
                        distance = np.sqrt(((1-limb.location)*sideLengths[limb.side])**2 +
                                           ((limbs[i+1].location)*sideLengths[limbs[i+1].side])**2 +
                                           (limb.length - limbs[i+1].length)**2)
                    while limb.size + limbs[i+1].size + 0.01 >= distance:
                        #make feet smaller and move them apart
                        if limb.size > limbs[i+1].size:
                            limb.size *= 0.81
                        elif limb.size < limbs[i+1].size:
                            limbs[i+1].size *= 0.81
                        else:
                            limb.size *= 0.9
                            limbs[i+1].size *= 0.9
                        limb.location = max(limb.location - limb.size/10, 0)
                        limbs[i+1].location = min(limbs[i+1].location + limbs[i+1].size/10, 
                                                  sideLengths[limbs[i+1].side])
                        lensign = 1 if (limb.length > limbs[i+1].length) else -1 #which way to change length
                        limb.length = limb.length + lensign*limb.size/10
                        limbs[i+1].length = limbs[i+1].length - lensign*limbs[i+1].size/10
                        #recalculate distance
                        if limbs[i+1].side == limb.side:
                            distance = np.sqrt(((limbs[i+1].location-limb.location)*sideLengths[limb.side])**2
                                               + (limb.length - limbs[i+1].length)**2)
                        else:
                            distance = np.sqrt(((1-limb.location)*sideLengths[limb.side])**2 +
                                               ((limbs[i+1].location)*sideLengths[limbs[i+1].side])**2 +
                                               (limb.length - limbs[i+1].length)**2)
                        reasonable = False
        if limbs[0].side == 0 and limbs[-1].side == 3: #check the last and first limbs don't border
            distance = (1-limbs[-1].location)*sideLengths[3]+limbs[0].location*sideLengths[0]
            while limbs[-1].size + limbs[0].size >= distance:
                #print 'last limb is intersecting'
                if limbs[-1].size > limbs[0].size:
                    limbs[-1].size *= 0.8
                elif limbs[-1].size < limbs[0].size:
                    limbs[0].size *= 0.8
                else:
                    limbs[-1].size *= 0.9
                    limbs[0].size *= 0.9
                limbs[-1].location -= limbs[-1].size/50.0
                limbs[0].location += limbs[0].size/50.0
                reasonable = False
    for limb in limbs:
        limb.mass = 10*(4/3.0*np.pi*limb.size**3)
    return Creature.Creature(bodySize, limbs)

def newRandomCreature():
    size = [random.random()*3 + .4, random.random()*3 + .4, random.random()*3 + .4]
    limbs = []
    for sideNum in range(4):
        for i in range(random.randint(0,3)):
            limbs.append(newRandomLimb(side = sideNum))
    return Creature.Creature(size, limbs)
simpleCreature = Creature.Creature(bodySize = [4,1.5,1], limbs = [Creature.Limb(1,.2,2,.3,),
                          Creature.Limb(1,.8,2,.3), Creature.Limb(3,.2,2,.3),
                          Creature.Limb(3,.8,2,.3)], name = "simpleCreature")