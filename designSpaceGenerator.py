import csv
import sys
import time
import Queue
import random
import numpy as np
import multiprocessing

from rf import SizedVehicle
from configobj import ConfigObj
from validate import Validator

poolSize = 10#0 #12000  # on my computer, I run about 25 cases/minute, or 1500/hour

inputs = (('Wing', 'SpanRadiusRatio'), ('Wing', 'WingAspectRatio'), ('Aux Propulsion', 'NumAuxProps'), ('Main Rotor', 'TaperRatio'), ('Main Rotor', 'TipTwist'), ('Main Rotor', 'Radius'), ('Main Rotor', 'TipSpeed'), ('Main Rotor', 'RootChord'), ('Main Rotor', 'NumBlades'))
inputRanges = ((0., 4.), (3., 9.), (0, 2), (.6, 1.), (-16, -4), (15., 35.), (400., 800.), (.5, 3.), (2, 6))


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
			if key not in ['Speeds', 'PowersCruise', 'PowersSL']:
				flatdict[key] = section[key]
		v.walk(flatten)
		return flatdict

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
	# load up the queue with jobs
	for i in xrange(poolSize):
		tasks.put(Task(v, m))

	print 'All tasks queued!'

	# put the endcaps on the queue to shut the workers down
	for i in xrange(numworkers):
		tasks.put(None)
	# get the results
	startTime = time.clock()
	output = {}
	for i in xrange(poolSize):
		if i>0: showProgress('design space', startTime, time.clock(), i, poolSize)
		flatdict = results.get()
		missingOutput = output.keys()
		for key, value in flatdict.iteritems():
			if key in output:
				output[key].append(value)
				missingOutput.remove(key)
			else:
				newvals = [0] * i
				newvals.append(value)
				output[key] = newvals
		for missing in missingOutput:
			output[missing].append(float('nan'))
	# join/close the tasks queue
	tasks.join()

	print 'Writing out the data!'
	# write out the data
	keys = output.keys()
	with open('Output/designSpace.csv', 'wb') as f:
		writer = csv.writer(f, delimiter=',')
		writer.writerow(keys)
		for i in xrange(poolSize):
			writer.writerow([output[key][i] for key in keys])





