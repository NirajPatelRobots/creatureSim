# -*- coding: utf-8 -*-
"""
animate the creatures
creates animations of creatures. The animations are of saved runs.
Requires VPython 7.5 or newer (https://vpython.org/)
Created Dec 2019 @author: Niraj

TODO:
    add a second light source so it isn't so dark on the back
    be more serious about error checking filenames (pretty easy)
    in animate_main, way to view information (active logs, annotations, files)
    make it easier to animate a single arrow/sphere from a row vector? (so you don't have to reshape to 2D)
    error checking for loading logs/ annotations
    way to remove annotations
    make creat.v destructors so creature animation gets deleted on error

"""
import vpython as v
import importlib
import pyximport
pyximport.install(language_level = 3)
import Creature
import numpy as np
import logger
import simulator

sidecolors = [v.color.yellow, v.color.white, v.color.cyan, v.color.orange]

floor = None
tlab = None

class AnimCreat:
    def __init__(self, origCreat):
        self.body = v.box(length = origCreat.bodySize[0], width = origCreat.bodySize[1],
                          height = origCreat.bodySize[2])
        self.eye1 = v.sphere(color = v.color.black, radius = origCreat.bodySize[1]/20)
        self.eye2 = v.sphere(color = v.color.black, radius = origCreat.bodySize[1]/20)
        self.limbs = []
        for olimb in origCreat.limbs:
            self.limbs.append(animLimb(olimb.size, olimb.side))
    def __del__(self):
        for limb in self.limbs:
            limb.trunk.visible = False
            del limb.trunk
            limb.foot.visible = False
            del limb.foot
        self.body.visible = False
        del self.body
        self.eye1.visible = False
        del self.eye1
        self.eye2.visible = False
        del self.eye2
        global tlab
        tlab.visible = False
        tlab = None

class animLimb:
    def __init__(self, size, side):
        self.trunk = v.cylinder(radius = size / 3)
        self.foot = v.sphere(radius = size, color = sidecolors[side])

def vCreateDisplay(creat): #make visualization possible
    v.scene.title = "Creature Animation"
    v.scene.forward = v.norm(v.vector(-.5,-.5,-1))
    v.scene.width = 800
    v.scene.height = 800
    v.scene.range = 6
    creat.v = AnimCreat(creat)

    #time label
    global tlab
    if tlab is None:
        tlab = v.label(text ='time:', height=15, border=3, color = v.color.white, line=False,
                       xoffset = v.scene.width/2 - 100, yoffset = -v.scene.height/2 + 50)
    #floor
    global floor
    if floor is None:
        floor = v.box(length = 10,width = 10, height = 0.01, color = v.color.green)

def animate(creature, fileName, speed=20, annotations = []): #make an animation, can't be done at the same time as real physics
    #Basically loads the saved creature movements, then moves it through them again, without physics, with visual
    print("Starting animation at speed", speed)
    
    v.sleep(1) #time for the file to be saved before loading
    if not fileName[-4:] == ".txt":   #ooh look @ me sanitizing those inputs
        fileName += ".txt"
    try:
        inFile = open("animations/" + fileName,'r')
    except:
        print("invalid file name")
        return
        
    #mess with number of lines and stuff
    maxLines = len(inFile.readlines()) # I don't care if this is slow
    inFile.seek(61)
    numTestpos = (len(inFile.readline().split())-3*(3+creature.numLimbs)-1)//6
    if numTestpos > 0:
        print("There are", numTestpos, "test arrows")
    inFile.seek(0)
    
    print("Select the animation and press e to end, p to pause", end='')
    #do animation
    for i, inLine in enumerate(inFile): #I don't care if enumerate is slow
        line = inLine.strip()
        if line.startswith("#"):
            continue
        if not i % speed == 0: #only play one in every "speed" frames
            continue
        splitline = line.split()
        #get data
        creatData = []
        for num in splitline[:3*(3+creature.numLimbs)]:
            creatData.append(float(num))
        
        for ann in annotations:
            ann.update(i * simulator.h)
        #set creature data
        pos = v.vector(creatData[0],creatData[1],creatData[2])
        creature.v.body.pos = pos
        axis = v.vector(creatData[3],creatData[4],creatData[5])
        creature.v.body.axis = creature.bodySize[0] * axis
        up = v.vector(creatData[6],creatData[7],creatData[8])
        creature.v.body.up = creature.bodySize[2] * up
        right = v.cross(axis, up)
        for olimb in creature.limbs:
            #################### find shoulders' positions ###########################
            location = pos - creature.bodySize[2] / 2 * up
            if olimb.side == 0:
                location += axis * creature.bodySize[0] / 2
                location += right * creature.bodySize[1] * (olimb.location - 0.5)
            elif olimb.side == 1:
                location += right * creature.bodySize[1] / 2
                location += -axis * creature.bodySize[0] * (olimb.location - 0.5)
            elif olimb.side == 2:
                location += -axis * creature.bodySize[0] / 2
                location += -right * creature.bodySize[1] * (olimb.location - 0.5)
            elif olimb.side == 3:
                location += -right * creature.bodySize[1] / 2
                location += axis * creature.bodySize[0] * (olimb.location - 0.5)
            #set limb data
            creature.v.limbs[olimb.num].foot.pos = v.vector(creatData[9+olimb.num*3],
                                                    creatData[10+olimb.num*3],creatData[11+olimb.num*3])
            creature.v.limbs[olimb.num].trunk.pos = location
            creature.v.limbs[olimb.num].trunk.axis = creature.v.limbs[olimb.num].foot.pos - location
        # face
        creature.v.eye1.pos = (pos + axis * creature.bodySize[0]/2 + up * creature.bodySize[2]/4 
                             + right * creature.bodySize[1]/8)
        creature.v.eye2.pos = creature.v.eye1.pos - right * creature.bodySize[1] / 4
        # time label
        tlab.text = "time: " + "{:.2f}".format(i * simulator.h) +'/' + "{:.2f}".format((maxLines - 1) * simulator.h)
        # key input
        keysdown = v.keysdown()
        if "e" in keysdown:
            break
        if "p" in keysdown:
            print("\r" + " "*60 + "\rpress r to resume", end='')
            while True:
                v.rate(100)
                if "r" in v.keysdown():
                    print("\rSelect the animation and press e to end, p to pause", end='')
                    break
        v.rate(100)
    print("\r" + " "*60 + "\r", end='')
    inFile.close()

def animate_main(creature, speed = None):
    fileName = "drop1" # by default
    if speed is None:
        speed = 10
    if not creature.v:
        vCreateDisplay(creature)
    annotations = []
    
    try:
        actLogger = logger.loadLogger(creature.name)
    except:
        print("Couldn't load logger", creature.name)
        actLogger = None
    else:
#        annotations.append(TestArrows(actLogger["limbPos"],
#                                      actLogger["walkForce"], scale = 0.03,
#                                        color = v.color.green))
#        annotations.append(TestSpheres(actLogger["desiredPoseCart"], color = v.color.red))e
        pass
    while True:
        args = input(">>>Enter animate (default), speed, load, log, arrows, spheres, or exit: ").split()
        command = args[0] if len(args) > 0 else ""
        if command == "speed" and len(args) > 1:
            speed = int(args[1])
        elif command == "load":
            fileName = input("\rEnter animation data filename: ")
            if len(args) > 1:
                fileName = args[1]
            elif fileName == "":
                fileName = "drop1"
            elif fileName == "exit":
                continue
        elif command == "log":
            loggerName = input("Enter saved Logger name: ")
            actLogger = logger.loadLogger(loggerName)
        elif command == "arrows":
            if len(args) > 1:
                locLogName = args[1]
            else:
                locLogName = input("Enter name of location log: ")
            if len(args) > 2:
                axisLogName = args[2]
            else:
                axisLogName = input("Enter name of direction log: ")
            if locLogName == "":
                locLog = None
            else:
                locLog = actLogger.l[locLogName]
            annotations.append(TestArrows(locLog, actLogger.l[axisLogName]))
        elif command == "spheres":
            if len(args) > 1:
                locLogName = args[1]
            else:
                locLogName = input("Enter name of location log: ")
            annotations.append(TestSpheres(actLogger.l[locLogName]))
        elif command == "exit":
            creature.v = None
            del annotations
            break
        else:
            animate(creature, fileName, speed, annotations)

class TestArrows:
    """ arrows that are added to an animation. 
    The data come from logs.
    """
    def __init__(self, locationLog, axisLog, color = v.color.green, scale = 1):
        """ where n is the number of arrows, T is the number of timesteps in the log
        locationLog is the ArrayLog (nx3xT) for where the arrows start.
            if locationLog is None, it will use the origin.
        axisLog is the ArrayLog (nx3xT) for where arrows point.
        color is a v.color for the arrows
        scale is a value to multiply the arrow lengths by
        """
        assert(locationLog is None or locationLog.d.shape[0] == axisLog.d.shape[0])
        assert(locationLog is None or locationLog.d.shape[1] == 3)
        assert(axisLog.d.shape[1] == 3)
        if axisLog.d.shape[2] == 0:
            print("WARNING: added a TestArrows with no data elements")
            if not locationLog is None:
                print(locationLog.printable())
            print(axisLog.printable())
        if locationLog is None:
            self.numObj = 1
        else:
            self.numObj = locationLog.d.shape[0]
        self.arrows = []
        self.locationLog = locationLog
        self.axisLog = axisLog
        self.scale = scale
        for i in range(self.numObj):
            self.arrows.append(v.arrow(color = color))
    
    def update(self, t):
        """updates the position of the arrows to time t"""
        if self.locationLog:
            loc_idx = (np.abs(self.locationLog.t - t)).argmin()
        axis_idx = (np.abs(self.axisLog.t - t)).argmin()
        for i in range(self.numObj):
            if self.locationLog:
                self.arrows[i].pos = np_to_v(self.locationLog[i, :, loc_idx])
            self.arrows[i].axis = np_to_v(self.axisLog[i, :, axis_idx] * self.scale)

    def __del__(self):
        for i in range(self.numObj):
            self.arrows[0].visible = False
            del self.arrows[0]

class TestSpheres:
    """ spheres that are added to animation. The data come from logs.
    Try not to call them "balls"
    data:
        color is a v.color object
        radius is radius of the spheres
    """
    def __init__(self, locationLog, color = v.color.blue, radius = 0.1):
        """locationLog is the ArrayLog (nx3) for where the spheres are.
        color is a v.color for the spheres
        radius is radius of the spheres
        """
        assert(locationLog.d.shape[1] == 3)
        self.numObj = locationLog.d.shape[0]
        self.spheres = []
        self.locationLog = locationLog
        for i in range(self.numObj):
            self.spheres.append(v.sphere(color = color, radius = radius))
            
    def update(self, t):
        """updates the position of the spheres to time t"""
        loc_idx = (np.abs(self.locationLog.t - t)).argmin()
        for i in range(self.numObj):
            self.spheres[i].pos = np_to_v(self.locationLog[i, :, loc_idx])
    
    def __del__(self):
        for i in range(self.numObj):
            self.spheres[0].visible = False
            del self.spheres[0]
            
def np_to_v(array):
    return v.vector(array[0], array[1], array[2])