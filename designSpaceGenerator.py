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

runTime = .4*60*60 # on my computer, I run about 25 cases/minute, or 1500/hour, and seem to get about 1 good case per minute out of it

inputs = (('Main Rotor', 'NumRotors'), ('Wing', 'SpanRadiusRatio'), ('Wing', 'WingAspectRatio'), ('Aux Propulsion', 'NumAuxProps'), ('Main Rotor', 'TaperRatio'), ('Main Rotor', 'TipTwist'), ('Main Rotor', 'Radius'), ('Main Rotor', 'TipSpeed'), ('Main Rotor', 'RootChord'), ('Main Rotor', 'NumBlades'))
inputRanges = ((1, 2), (0., 4.), (3., 9.), (0, 1), (.6, 1.), (-16, -4), (15., 35.), (400., 800.), (.5, 3.), (2, 6))


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
        flatdict['COMPUTERNAME'] = os.environ['COMPUTERNAME']
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

if __name__ == '__main__':
    startTime = time.clock()
    # write the "I am currently running" file
    runFile = 'Output/running_%s' % os.environ['COMPUTERNAME']
    with open(runFile, 'w') as f: f.write('blah')
    # ok now let's start the actual stuff
    v = ConfigObj('Config/vehicle.cfg', configspec='Config/vehicle.configspec')
    m = ConfigObj('Config/mission_singlesegment.cfg', configspec='Config/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    tasks = multiprocessing.JoinableQueue()
    results = multiprocessing.Queue()
    numworkers = multiprocessing.cpu_count() * 2
    workers = [Worker(tasks, results) for i in xrange(numworkers)]
    for w in workers:
        w.start()

    # find our output file name
    fnum = 0
    fileNameTemplate = 'Output/designSpace_%s_%d.csv'
    fileName = fileNameTemplate % (os.environ['COMPUTERNAME'], fnum)
    while os.path.isfile(fileName):
        fnum += 1
        fileName = fileNameTemplate % (os.environ['COMPUTERNAME'], fnum)
    startTime = time.time()
    endTime = startTime + runTime
    # keep looping until we actually get our first real result
    gotKeys = False
    goodRows = 0
    totalRows = 0
    outstandingTasks = 1
    tasks.put(Task(v, m))
    with open(fileName, 'wb') as f:
        while outstandingTasks > 0:
            showProgress('%d good, %d total, %d outstanding' % (goodRows, totalRows, outstandingTasks), startTime, time.time(), endTime)
            if outstandingTasks<multiprocessing.cpu_count()*2 and time.time()<endTime:
                for i in xrange(multiprocessing.cpu_count()):
                    tasks.put(Task(v, m))
                    outstandingTasks += 1
            flatdict = results.get()
            outstandingTasks -= 1
            totalRows += 1
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
                    # update the runfile with current status
                    os.remove(runFile)
                    currentTime = time.time()
                    runFile = 'running_%s   %d good     %.2f per hr     %d m remaining' % (os.environ['COMPUTERNAME'], goodRows, goodRows/(currentTime-startTime)*60*60, (endTime-startTime)/60
                    with open(runFile, 'w') as f: f.write('blah')
    for i in xrange(numworkers):
        tasks.put(None)
    # join/close the tasks queue
    tasks.join()
    # remove the "I'm currently running" file
    if os.path.isfile(runFile):
        os.remove(runFile)

    # print 'Writing out the data!'
    #     for i in xrange(len(output[keys[0]])):
    #         writer.writerow([output[key][i] for key in keys])