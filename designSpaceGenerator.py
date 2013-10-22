# CONDOR
# Copyright (C) 2013 Michael Roberts

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


import os
import csv
import sys
import time
import random
import numpy as np
import multiprocessing

from rf import SizedVehicle
from vehicle import Vehicle
from configobj import ConfigObj
from validate import Validator

# BEGIN SCRIPT CONFIG BLOCK
vehicleConfigPath = 'Config/vehicle_s92.cfg'
missionConfigPath = 'Config/mission_singlesegment.cfg'
inputs = (('Aux Propulsion', 'NumAuxProps'), ('Main Rotor', 'DiskLoading'), ('Main Rotor', 'Solidity'), ('Main Rotor', 'TipSpeed'))  # Config blocks and items to vary
inputRanges = ((0, 1), (2., 30.), (.05, .15), (400., 800.)) # Min and max for each config item, respectively
missionInputs = (('Segment 2', 'Distance'), ('Segment 2', 'Speed'))  # Mission blocks and items to vary
missionInputRanges = ((300., 1000.), (150., 300.))  # Min and max for each mission config item, respectively

if os.getcwd().split(os.sep)[-1] == 'CONDOR':
    runFileFolder = 'Output/RunFiles/'
    outputFolder = 'Output/'
else:
    runFileFolder = '../CONDOR/Output/RunFiles/'
    outputFolder = '../CONDOR/Output/'
idleFile = 'AAA_Keep_Idling_ALL'
activelyRunFile = 'AAA_Computations_Active_ALL'
killFilePrefix = 'AAAA_FORCEKILL_'
forceIdleFilePrefix = 'AAAA_FORCEIDLE_'
inUseFilePrefix = 'AAAA_InUse_'

idleSleepTime = 10
minRunFileUpdateTime = 60 # minimum number of seconds between runfile updates
# END CONFIG BLOCK

lastRunFileUpdateTime = 0
startTime = time.time() - 1 # subtract one second so we don't divide by zero on the first update

computerName = ''
try:
    computerName = os.uname()[1]
except:
    computerName = os.environ['COMPUTERNAME']
runFile = '%s%s %s' % (runFileFolder, computerName, 'quit')
killFile = killFilePrefix + computerName
forceIdleFile = forceIdleFilePrefix + computerName
inUseFile = inUseFilePrefix + computerName
lastState = 'quit'

class Worker(multiprocessing.Process):

    def __init__(self, task_queue, result_queue):
        multiprocessing.Process.__init__(self)
        self.tasks = task_queue
        self.results = result_queue

    def run(self):
        while True:
            task = self.tasks.get()
            if task is None:
                self.tasks.task_done()
                break
            flatdict = task()
            self.tasks.task_done()
            self.results.put(flatdict)

class Task(object):
    def __init__(self, vconfig, mconfig, airfoildata_mainRotor, airfoildata_auxProp, gw=30000):
        self.vconfig = vconfig
        self.mconfig = mconfig
        self.airfoildata_mainRotor = airfoildata_mainRotor
        self.airfoildata_auxProp = airfoildata_auxProp
        self.gw = gw
    def __call__(self):
        vconfig = ConfigObj(self.vconfig)
        mconfig = ConfigObj(self.mconfig)
        for i in xrange(len(inputs)):
            if type(vconfig[inputs[i][0]][inputs[i][1]]) is int:
                val = random.randint(inputRanges[i][0], inputRanges[i][1])
            else:
                val = random.uniform(inputRanges[i][0], inputRanges[i][1])
            vconfig[inputs[i][0]][inputs[i][1]] = val
        for i in xrange(len(missionInputs)):
            if type(mconfig[missionInputs[i][0]][missionInputs[i][1]]) is int:
                val = random.randint(missionInputRanges[i][0], missionInputRanges[i][1])
            else:
                val = random.uniform(missionInputRanges[i][0], missionInputRanges[i][1])
            mconfig[missionInputs[i][0]][missionInputs[i][1]] = val
        #vehicle = SizedVehicle(vconfig, mconfig, self.airfoildata_mainRotor, self.airfoildata_auxProp)
        #sizedVehicle = vehicle.sizeMission() # this is now a Vehicle object, or False if it wasn't able to converge properly.
        sizedVehicle = Vehicle(vconfig, mconfig, self.gw, self.airfoildata_mainRotor, self.airfoildata_auxProp)
        if sizedVehicle:
            sizedVehicle.generatePowerCurve()
            sizedVehicle.scaleEngine()
            sizedVehicle.findCost()
            sizedVehicle.findHoverCeiling()
            sizedVehicle.findMaxRange()
            sizedVehicle.findMaxSpeed()
            sizedVehicle.findMaxEndurance()
            sizedVehicle.OEC()
            v = sizedVehicle.vconfig
            flatdict = {}
            def flatten(section, key):
                if section.name not in ['Condition', 'Trim Failure', 'Simulation']:
                    flatdict[key] = section[key]
            v.walk(flatten)
            flatdict['COMPUTERNAME'] = computerName
            if flatdict['Payload']>0 and flatdict['MaxRange']>0:
                return flatdict
            else:
                return None
        else:
            return None

def showProgress(name, startTime, currentTime, endTime):
    """Shows progress information in the terminal."""
    elapsedTime = currentTime - startTime
    remainingTime = endTime - currentTime
    totalTime = endTime - startTime
    percentage = elapsedTime / totalTime
    barlen = 80
    barfill = (int) (round(barlen * percentage))
    bar = ''.rjust(barfill,'\xFE').ljust(barlen, '.')
    s = '     %-40s     %s of %s     %s remaining %90s' % (name, fmtTime(elapsedTime), fmtTime(totalTime), fmtTime(remainingTime), bar)
    sys.stdout.write(s)
    sys.stdout.flush()
    sys.stdout.write("\b" * len(s))

def fmtTime(total):
    hours = (int) (total / 60 / 60)
    minutes = (int) (total / 60 - hours*60)
    seconds = (int) (total - hours*60*60 - minutes*60)
    return '%02d:%02d:%02d' % (hours, minutes, seconds)

def updateStatus(state, goodRows=0, totalRows=0, outstandingTasks=1):
    """Called when the status changes.  Prints it to the terminal and writes out a new runFile."""
    global runFile
    global startTime
    global lastRunFileUpdateTime
    global computerName
    global minRunFileUpdateTime
    elapsedTime = time.time() - startTime
    if state == 'starting':
        status = 'starting'
    elif state == 'idling':
        status = 'idling'
    elif state == 'running':
        status = 'running        %dg        %dt        %dgph        %dtph' % (goodRows, totalRows, goodRows/elapsedTime*60*60, totalRows/elapsedTime*60*60)
    elif state == 'stopping':
        status = 'stopping        %d tasks remaining' % (outstandingTasks)
    elif state == 'quit':
        status = 'quit'
    statusLine = '%-20s %-60s %30s' % (computerName, status, time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()))
    print(statusLine)
    updateElapsedTime = time.time() - lastRunFileUpdateTime
    if updateElapsedTime>minRunFileUpdateTime or lastState!=state:
        if os.path.isfile(runFile): os.remove(runFile)
        runFile = '%s%s %s' % (runFileFolder, computerName, status)
        with open(runFile, 'w') as f: f.write('blah')
        lastRunFileUpdateTime = time.time()





if __name__ == '__main__':

    tasks = multiprocessing.JoinableQueue()
    results = multiprocessing.Queue()
    if not os.path.isfile(runFileFolder + idleFile):

        print('idleFile not found, so I will not run!  Create the file %s%s to keep idling, plus %s%s to run.' % (runFileFolder, idleFile, runFileFolder, activelyRunFile))

    while os.path.isfile(runFileFolder+idleFile) and not os.path.isfile(runFileFolder + killFile):
        if os.path.isfile(runFileFolder+activelyRunFile) and not os.path.isfile(runFileFolder+killFile)  and not os.path.isfile(runFileFolder+forceIdleFile):
            v = ConfigObj(vehicleConfigPath, configspec='Config/vehicle.configspec')
            m = ConfigObj(missionConfigPath, configspec='Config/mission.configspec')
            vvdt = Validator()
            v.validate(vvdt)
            mvdt = Validator()
            m.validate(mvdt)
            c81File_mainRotor = 'Config/%s'%v['Main Rotor']['AirfoilFile']
            c81File_auxProp = 'Config/%s'%v['Aux Propulsion']['AirfoilFile']
            airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
            airfoildata_auxProp = np.genfromtxt(c81File_auxProp, skip_header=0, skip_footer=0) # read in the airfoil file
            updateStatus('starting')
            if os.path.isfile(runFileFolder+inUseFile):
                numworkers = multiprocessing.cpu_count() - 1
            else:
                numworkers = multiprocessing.cpu_count() * 2
            workers = [Worker(tasks, results) for i in xrange(numworkers)]
            for w in workers:
                w.start()

            # find our output file name
            fnum = 0
            fileNameTemplate = outputFolder + 'designSpace_%s_%d.csv'
            fileName = fileNameTemplate % (computerName, fnum)
            while os.path.isfile(fileName):
                fnum += 1
                fileName = fileNameTemplate % (computerName, fnum)
            startTime = time.time() - 1 # minus one second so that we don't divide by zero on the first loop
            # keep looping until we actually get our first real result
            gotKeys = False
            goodRows = 0
            totalRows = 0
            keepRunning = True
            outstandingTasks = 1
            tasks.put(Task(v, m, airfoildata_mainRotor, airfoildata_auxProp))
            with open(fileName, 'wb') as f:
                while outstandingTasks>0:
                    # check if we should stop
                    if os.path.isfile(runFileFolder+killFile) or os.path.isfile(runFileFolder+forceIdleFile) or not os.path.isfile(runFileFolder+activelyRunFile): keepRunning = False
                    # add tasks to the queue if we're running low
                    if outstandingTasks<numworkers and keepRunning:
                        for i in xrange(numworkers):
                            gw = random.uniform(10000, 60000)
                            tasks.put(Task(v, m, airfoildata_mainRotor, airfoildata_auxProp, gw))
                            outstandingTasks += 1
                    # update status output
                    updateStatus('running' if keepRunning else 'stopping', goodRows, totalRows, outstandingTasks=outstandingTasks)
                    # get a results
                    flatdict = results.get()
                    outstandingTasks -= 1
                    totalRows += 1
                    # write out result if it's good
                    if flatdict is not None:
                        goodRows += 1
                        if gotKeys:
                            writer.writerow(flatdict)
                        else:
                            keys = flatdict.keys()
                            writer = csv.DictWriter(f, keys, delimiter=',')
                            writer.writerow({key:key for key in keys})
                            writer.writerow(flatdict)
                            f.flush()
                            gotKeys = True
            for i in xrange(numworkers):
                tasks.put(None)
        updateStatus('idling')
        time.sleep(idleSleepTime)

    # join/close the tasks queue
    tasks.join()
    updateStatus('quit')