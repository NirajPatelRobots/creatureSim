"""
file for Creature for logging information.
The logged data aren't saved runs, it's other interesting data for debugging and monitoring.
This isn't used for regular functionality, only development.
Created on Wed Apr 1 2020

@author: Niraj

################### global variables ################
allEnabled - whether logging should happen. Disabling it makes all of the functions
    and methods in this library (other than class constructors) have no effects.
allTimingEnabled - if allTimingEnabled is False, the logger will not log
    any system times as irltime, regardless of log settings (default = True)
allLoggers - a dictionary of all current Logger and subclass objects,
    keys = names and values = objects
tOffset (float) - the time in [s] it takes to start and end the timer. Can 
        be used to offset a measured system time from time.clock().
"""
"""
TODO:
    finish developing DictLog
    make a decorator for locking a function (does one exist?)
    make Logs subclasses of a superclass Log
    maybe look into using python's logger class
    make it so you can set a recommended view scale
    make it always run in another thread?
    make a reset for all logs in a logger
"""
import numpy as np
import os.path
import pickle
import time
import threading
from collections import defaultdict

allEnabled = True
allTimingEnabled = True
allLoggers = {}

nTimeCheckIter = 10
tOffset = 0
for i in range(nTimeCheckIter):
    tstart = time.clock()
    tOffset += time.clock() - tstart
tOffset /= nTimeCheckIter

class ArrayLog:
    """ an ArrayLog is one set of data for a Logger object.
    It stores data in numpy arrays.
    
    -- Attributes
    name (str) - the name of the data
    loggerName (str) - the name of the attached logger
    enabled (bool) - data are logged only if enabled == True (default = True)
    timingEnabled - whether the logger saves system times (default = False)
    d (array) - the data, which has the same shape as self.shape but with a new
        last axis with length T (T = the number of logged data points)
    t (1D np.array float) - the simulation time of this point in seconds
    tirl (1D np.array float) - the system time from time.time() 
        if timing is not enabled, self.tirl will not be updated (see timingEnabled)
    lock - threading.Lock() object for locking the data
    """
    def __init__(self, name, loggerName, enabled = True, timingEnabled = False,
                 shape = None, dtype = None):
        """constructor.
        shape is the shape of each individual logged point (default = scalar).
        dtype is the data type of the array.
        """
        self.name = name
        self.loggerName = loggerName
        self.enabled = enabled
        self.timingEnabled = timingEnabled
        self._ptdtype = dtype
        self.setShape(shape, self._ptdtype)
        self.lock = threading.Lock()
        self.t = np.array([])
        self.tirl = np.array([])
        
    def setShape(self, shape = None, dtype = None):
        """ sets the shape of every individual logged point
        this method also clears the ArrayLog's data.
        this method is not affected by global allEnabled or 
        -- Parameters
        shape is the shape of every point (default = scalar)
        dtype sets the datatype of the array (default = numpy default)
        """
        #add a 0 to the end of the shape for the dimension that iterates points
        if shape == None:
            self._shape = np.array([0])
        else:
            self._shape = np.array([x for x in shape]+[0])
        self.d = np.zeros(self._shape, dtype = dtype)
        
    
    def addPoint(self, point, tirl = None):
        """ adds a data point.
        -- Parameters
        point is the array of shape self._shape that should be appended to self.d
        tirl is the current real time, if known
        
        returns the point that was added.
        """
        global allLoggers
        t = allLoggers[self.loggerName].t
        if(not self.lock.locked()): #if the lock is unused
            self._addPointThread(point, t, tirl)
        else: #if the lock is taken, make a new thread that will wait
            t = threading.Thread(target = self._addPointThread, args = (point, t, tirl))
            t.start()
        return point
            
    def _addPointThread(self, point, t, tirl = None):
        global allEnabled
        if allEnabled and self.enabled and (self.t.size == 0 or self.t[-1] < t):
            self.lock.acquire(True) #wait for the lock
            self.d = np.dstack((self.d, point))
            self.t = np.append(self.t, t)
            global allTimingEnabled
            if not tirl == None:
                self.tirl = np.append(self.tirl, tirl)
            elif (allTimingEnabled and self.timingEnabled):
                self.tirl = np.append(self.tirl, time.time())
            self.lock.release()
    
    def __getitem__(self, index):
        return self.d[index]
                
    def printable(self, printData = True, print_t = False, print_tirl = False):
        """ returns a string describing this ArrayLog.
        if printData is True, it will also print the value of the logged data d.
        print_t and print_tirl control whether t and tirl are printed, if they aren't empty.
        """
        p = ("Logged array " + self.name + "\n"
             + "\tPoint shape: "+str(self._shape[:-1])
             + "  Number of points: "+str(self._shape[-1]) + "\n")
        if self.t.size > 0:
            p += ("Simulation time: " + str(self.t.size) + " points, range "
                  + str(np.min(self.t)) + " to " + str(np.max(self.t)) + "\n")
        if print_t:
            p += "\tt:" + str(self.t).replace("\n", "\n\t")
        if self.tirl.size > 0:
            p += ("Real world time: " + str(self.tirl.size) + " points, range "
                  + str(np.min(self.tirl)) + " to " + str(np.max(self.tirl)) + "\n")
        if print_tirl:
            p += "\ttirl:\n\t" + str(self.tirl).replace("\n", "\n\t")
        if printData:
            p += "\t" + str(self.d).replace("\n", "\n\t")
        p.rstrip()
        return p
    
    def clear(self):
        """ removes all the logged data from a ArrayLog.
        Does not change the time logging settings or point shape.
        """
        self.d = np.array(self._shape, dtype = self._ptdtype)
    
    def __getstate__(self):
        """ method used by pickle. removes the lock, which can't be pickled."""
        d = self.__dict__.copy()
        if 'lock' in d:
            del d['lock']
        return d
    
    def __setstate__(self, d):
        self.__dict__.update(d)
        self.lock = threading.Lock()
             
        
class DictLog:
    """ a DictLog is one set of data for a Logger object.
    It stores data in a python defaultdict.
    
    -- Attributes
    name (str) - the name of the data
    loggerName (str) - the name of the attached logger
    enabled (bool) - data are logged only if enabled == True (default = True)
    d (defaultdict) - the data
    lock - threading.Lock() object for locking the data
    """
    def __init__(self, name, loggerName, enabled = True, default = list,
                 initdata = None):
        """ Constructor. 
        default is the default_factory of the defaultdict (default = list)
        initdata is an optional mapping or iterable to form the defaultdict.
        """
        self.name = name
        self.loggerName = loggerName
        self.enabled = enabled
        self.lock = threading.Lock()
        if initdata == None:
            self.d = defaultdict(default)
        else:
            self.d = defaultdict(default, initdata)
            
    def addPoint(self, key, value, behavior = 'append'):
        """ adds a data point. Locks self.lock while writing.
        -- Parameters
        key is the key in the default_dict to affect. A new entry is created if
            it doesn't exist.
        value is the new value for self.d[key]. Its behavior is governed by behavior.
        behavior - can be one of:
            'append': self.d[key].append(value)
            'add': self.d[key] += value
            'mult': self.d[key] *= value
        """
        if(not self.lock.locked()):
            self._addPointThread(key, value, behavior)
        else:
            t = threading.Thread(target = self._addPointThread, args = (key, value, behavior))
            """throw the thread into the wind.
            You do not see where it lands. The reference is lost when this method returns.
            But as you toss it into the air and blow to help it catch the breeze,
            you see it float, as if weightless, searching for its own free patch of your CPU.
            Before you turn and exit this method, a peaceful feeling washes over you
            which lasts for the rest of your day.
            """
            t.start()
            
    def _addPointThread(self, key, value, behavior):
        global allEnabled
        if allEnabled and self.enabled:
            self.lock.acquire(True) #wait for the lock
            if behavior == 'append':
                self.d[key].append(value)
            elif behavior == 'add':
                self.d[key] += value
            elif behavior == 'mult':
                self.d[key] *= value
            else:
                print("behavior:", behavior, "is invalid")
            self.lock.release()
    
    def printable(self, printData = True):
        """ returns a string describing this DictLog.
        if printData is True, it will also include the value of the logged data d.
        """
        p = ("Logged dict " + self.name + "\n"
             + "\tNumber of Keys: "+str(len(self.d.keys()))
             + "  Number of points: "+str(len(self.d.values())) + "\n")
        if printData:
            p += "\tData:\n\t" + str(self.d).replace("\n", "\n\t")
        p.rstrip()
        return p
    
    def __getitem__(self, index):
        return self.d[index]
    
    def clear(self):
        self.d.clear() #yeehaw
        
    def __getstate__(self):
        """ method used by pickle. removes the lock, which can't be pickled."""
        d = self.__dict__.copy()
        if 'lock' in d:
            del d['lock']
        return d
    
    def __setstate__(self, d):
        self.__dict__.update(d)
        self.lock = threading.Lock()

class Logger:
    """ a Logger is a threadsafe class for logging data.
    Attributes
    ownerType (str) - whether the owner is a Creature, Simulator, or other
    ownerName (str) - name of the owner
    logs (dict) - dictionary for each Log. self.logs['logName'] = <Log Object>
    t (float) - time index for taking logs. if t isn't updated, ArrayLogs in
        this logger won't add points.
    """
    def __init__(self, ownerName = None, ownerType = None):
        self.ownerType = ownerType
        self.ownerName = ownerName
        self.l = {}
        self.t = -1.
        global allLoggers
        allLoggers[self.ownerName] = self
        
    def __getitem__(self, index):
        return self.l[index]
        
    def newArrayLog(self, logName, enabled = True, timingEnabled = False,
                    shape = None, dtype = None):
        """ Creates a new ArrayLog named logName. 
        Completely overrides any existing log with that name.
        """
        #print(self.ownerName, "making ArrayLog", logName)
        self.l[logName] = ArrayLog(logName, self.ownerName, enabled, 
                                   timingEnabled, shape, dtype)
        
    def newDictLog(self, logName, enabled = True, default = list,
                 initdata = None):
        """ Creates a new DictLog named logName. 
        Completely overrides any existing log with that name.
        """
        self.l[logName] = DictLog(logName, self.ownerName, enabled,
                                  default, initdata)
            
    def save(self, fileName = None, loud = False):
        """ save the Logger data to a pickle file
            fileName is a string. Default is self.ownerName.
                File is saved to logs/fileName.creatlog.
            loud is whether it prints a message
        """
        if fileName is None:
            fileName = self.ownerName
        if loud:
            print("Saving "+ self.printable())
        fileName = os.path.join("logs", fileName + ".creatlog")
        
        with open(fileName, 'wb') as outFile:
            pickle.dump(self, outFile, 0)
    
    def printable(self):
        string = "Logger " + self.ownerName + "\n"
        for log in self.l.values():
            string += log.printable(printData = False)
        return string
    
    

def loadLogger(fileName, loud = False):
    """returns a log saved by Logger.save() """
    fileName = os.path.join("logs", fileName + ".creatlog")
    
    with open(fileName, 'rb') as inFile:
        logger = pickle.load(inFile)
    if loud:
        print("loaded", fileName, ":", logger.printable())
    return logger
    
        