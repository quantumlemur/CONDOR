import os
import csv
import sys
import time
import random
import numpy as np
import multiprocessing

from rf import SizedVehicle
from configobj import ConfigObj
from validate import Validator

runTime = 1*60*60 # on my computer, I run about 25 cases/minute, or 1500/hour, and seem to get about 1 good case per minute out of it

inputs = (('Main Rotor', 'NumRotors'), ('Wing', 'SpanRadiusRatio'), ('Wing', 'WingAspectRatio'), ('Aux Propulsion', 'NumAuxProps'), ('Main Rotor', 'TaperRatio'), ('Main Rotor', 'TipTwist'), ('Main Rotor', 'Radius'), ('Main Rotor', 'TipSpeed'), ('Main Rotor', 'RootChord'), ('Main Rotor', 'NumBlades'))
inputRanges = ((1, 2), (0., 4.), (3., 9.), (0, 1), (.6, 1.), (-16, -4), (15., 35.), (400., 800.), (.5, 3.), (2, 6))

runFileFolder = 'Output/RunFiles/'
outputFolder = 'Output/'
idleFile = 'AAAA_Idle'
activelyRunFile = 'AAAA_Run'
killFilePrefix = 'AAAA_KILL_'


idleSleepTime = 10
minRunFileUpdateTime = 60 # minimum number of seconds between runfile updates

lastRunFileUpdateTime = 0
startTime = time.time() - 1 # subtract one second so we don't divide by zero on the first update

computerName = ''
try:
    computerName = os.uname()[1]
except:
    computerName = os.environ['COMPUTERNAME']
runFile = ''
killFile = killFilePrefix + computerName

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
    def __init__(self, vconfig, mconfig):
        self.vconfig = vconfig
        self.mconfig = mconfig
    def __call__(self):
        vconfig = ConfigObj(self.vconfig)
        mconfig = ConfigObj(self.mconfig)
        for i in xrange(len(inputs)):
            if type(vconfig[inputs[i][0]][inputs[i][1]]) is int:
                val = random.randint(inputRanges[i][0], inputRanges[i][1])
            else:
                val = random.uniform(inputRanges[i][0], inputRanges[i][1])
            vconfig[inputs[i][0]][inputs[i][1]] = val
        vehicle = SizedVehicle(vconfig, mconfig)
        sizedVehicle = vehicle.sizeMission() # this is now a Vehicle object
        sizedVehicle.generatePowerCurve()
        sizedVehicle.findHoverCeiling()
        sizedVehicle.findMaxRange()
        sizedVehicle.findMaxSpeed()
        v = sizedVehicle.vconfig
        flatdict = {}
        def flatten(section, key):
            if section.name not in ['Condition', 'Trim Failure']:
                flatdict[key] = section[key]
        v.walk(flatten)
        flatdict['COMPUTERNAME'] = computerName
        if flatdict['GoodRun']:
            return flatdict
        else:
            return None

def showProgress(name, startTime, currentTime, endTime):
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
        status = 'done, halting'
    statusLine = '%-20s %-60s %30s' % (computerName, status, time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()))
    print(statusLine)
    updateElapsedTime = time.time() - lastRunFileUpdateTime
    if state == 'quit':
        if os.path.isfile(runFile): os.remove(runFile)
    elif updateElapsedTime > minRunFileUpdateTime:
        if os.path.isfile(runFile): os.remove(runFile)
        runFile = '%s%s %s        %s' % (runFileFolder, computerName, status, time.strftime("%H.%M.%S", time.localtime()))
        with open(runFile, 'w') as f: f.write('blah')
        lastRunFileUpdateTime = time.time()





if __name__ == '__main__':
    v = ConfigObj('Config/vehicle.cfg', configspec='Config/vehicle.configspec')
    m = ConfigObj('Config/mission_singlesegment.cfg', configspec='Config/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    tasks = multiprocessing.JoinableQueue()
    results = multiprocessing.Queue()
    if not os.path.isfile(runFileFolder + idleFile):
        print('idleFile not found, so I will not run!  Create the file %s%s to keep idling, plus %s%s to run.' % (runFileFolder, idleFile, runFileFolder, activelyRunFile))

    while os.path.isfile(runFileFolder + idleFile) and not os.path.isfile(runFileFolder + killFile):
        if os.path.isfile(runFileFolder + activelyRunFile) and not os.path.isfile(runFileFolder + killFile):
            updateStatus('starting')
            numworkers = 3 if computerName=='LYNX' else multiprocessing.cpu_count()*2
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
            tasks.put(Task(v, m))
            with open(fileName, 'wb') as f:
                while outstandingTasks>0:
                    # check if we should stop
                    if os.path.isfile(runFileFolder + killFile) or not os.path.isfile(runFileFolder + activelyRunFile): keepRunning = False
                    # add tasks to the queue if we're running low
                    if outstandingTasks<numworkers and keepRunning:
                        for i in xrange(numworkers):
                            tasks.put(Task(v, m))
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

    # print 'Writing out the data!'
    #     for i in xrange(len(output[keys[0]])):
    #         writer.writerow([output[key][i] for key in keys])
