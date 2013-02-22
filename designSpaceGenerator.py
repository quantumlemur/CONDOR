import sys
import time
import Queue
import random
import numpy as np
import multiprocessing

from rf import SizedVehicle
from configobj import ConfigObj
from validate import Validator

poolSize = 100

inputs = (('Main Rotor', 'Radius'), ('Main Rotor', 'TipSpeed'), ('Main Rotor', 'RootChord'))
inputRanges = ((20., 35.), (400., 800.), (.5, 3.))


class Worker(multiprocessing.Process):
    
    def __init__(self, task_queue, result_queue):
        multiprocessing.Process.__init__(self)
        self.tasks = task_queue
        self.results = result_queue
        
    def run(self):
        while True:
            task = self.tasks.get()
            if task is None:
                print "process exiting\n"
                self.tasks.task_done()
                break
            kid = task()
            self.tasks.task_done()
            self.results.put(kid)

class Task(object):
	def __init__(self, vconfig, mconfig):
		self.vconfig = ConfigObj(vconfig)
		self.mconfig = ConfigObj(mconfig)
	def __call__(self):
		vehicle = SizedVehicle(self.vconfig, self.mconfig)
        sizedVehicle = vehicle.sizeMission() # this is now a Vehicle object
	    sizedVehicle.generatePowerCurve()
	    sizedVehicle.findHoverCeiling()
	    sizedVehicle.findMaxRange()
	    sizedVehicle.findMaxSpeed()
	    return sizedVehicle.vconfig

def showProgress(name, startTime, currentTime, currentRow, totalRows):
    elapsedTime = currentTime - startTime
    timePerRow = elapsedTime / currentRow
    remainingRows = totalRows - currentRow
    remainingTime = remainingRows * timePerRow
    totalTime = totalRows * timePerRow
    percentage = elapsedTime / totalTime
    barlen = 80
    barfill = (int) (round(barlen * percentage))
    bar = ''.rjust(barfill,'\xFE').ljust(barlen, '.')
    s = '     %-40s     %3d of %3d      %s of %s     %s remaining %90s' % (name, currentRow, totalRows, fmtTime(elapsedTime), fmtTime(totalTime), fmtTime(remainingTime), bar)
    sys.stdout.write(s)
    sys.stdout.flush()
    sys.stdout.write("\b" * len(s))

def fmtTime(total):
    hours = (int) (total / 60 / 60)
    minutes = (int) (total / 60)
    seconds = (int) (total - minutes*60)
    return '%02d:%02d:%02d' % (hours, minutes, seconds)

if __name__ == '__main__':
	startTime = time.clock()
	v = ConfigObj('Config/vehicle.cfg', configspec='Config/vehicle.configspec')
	m = ConfigObj('Config/mission.cfg', configspec='Config/mission.configspec')
	vvdt = Validator()
	v.validate(vvdt)
	mvdt = Validator()
	m.validate(mvdt)





	# tasks = multiprocessing.JoinableQueue()
 #    results = multiprocessing.Queue()
 #    numworkers = multiprocessing.cpu_count() * 2
 #    workers = [Worker(tasks, results) for i in xrange(numworkers)]
 #    for w in workers:
 #        w.start()
