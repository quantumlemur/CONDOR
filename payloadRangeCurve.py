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



import sys
import time
import signal
import numpy as np
import multiprocessing
import matplotlib.pylab as pylab
import matplotlib.pyplot as plt

from vehicle import Vehicle
from configobj import ConfigObj
from validate import Validator

# BEGIN SCRIPT CONFIG BLOCK
numPerAxis = 20
vehicleConfigPath = 'Config/vehicle_s92.cfg'
missionConfigPath = 'Config/AHS_mission%d.cfg'
rangeMin = 0
rangeMax = 1000
payloadMin = 0
payloadMax = 12000
# END SCRIPT CONFIG BLOCK



mission = 1
halted = False
tasks = multiprocessing.JoinableQueue()
queuedtasks = []
results = multiprocessing.Queue()
ranges = np.linspace(rangeMin, rangeMax, numPerAxis)
payloads = np.linspace(payloadMin, payloadMax, numPerAxis)
startTime = time.time() - 1 # subtract one second so we don't divide by zero on the first update

def signal_handler(signal, frame):
    global halted
    print('HALTING')
    halted = True
    #tasks.clear()

signal.signal(signal.SIGINT, signal_handler)

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
            (vconfig, i, j) = task()
            self.tasks.task_done()
            self.results.put((vconfig, i, j))

class Task(object):
    def __init__(self, vconfig, mconfig, airfoildata_mainRotor, airfoildata_auxProp, range, payload, i, j):
        self.vconfig = vconfig
        self.mconfig = mconfig
        self.airfoildata_mainRotor = airfoildata_mainRotor
        self.airfoildata_auxProp = airfoildata_auxProp
        self.payload = payload
        self.range = range
        self.i = i
        self.j = j
    def __call__(self):
        vconfig = ConfigObj(self.vconfig)
        mconfig = ConfigObj(self.mconfig)
        mconfig['Segment 2']['Distance'] = self.range
        mconfig['Segment 4']['Distance'] = self.range
        mconfig['Segment 1']['PayloadWeight'] = self.payload
        mconfig['Segment 2']['PayloadWeight'] = self.payload
        mconfig['Segment 3']['PayloadWeight'] = self.payload
        mconfig['Segment 4']['PayloadWeight'] = self.payload
        vehicle = Vehicle(vconfig, mconfig, 34000., self.airfoildata_mainRotor, self.airfoildata_auxProp)
        vehicle.flyMission()
        return (vehicle.vconfig, self.i, self.j)

def showProgress(startTime, currentTime, currentRow, totalRows):
    if currentRow==0:  currentRow = 1
    elapsedTime = currentTime - startTime
    timePerRow = elapsedTime / currentRow
    remainingRows = totalRows - currentRow
    remainingTime = remainingRows * timePerRow
    totalTime = totalRows * timePerRow
    percentage = elapsedTime / totalTime
    barlen = 80
    barfill = (int) (round(barlen * percentage))
    bar = ''.rjust(barfill,'\xFE').ljust(barlen, '.')
    s = '%3d of %3d      %s of %s     %s remaining %90s' % (currentRow, totalRows, fmtTime(elapsedTime), fmtTime(totalTime), fmtTime(remainingTime), bar)
    sys.stdout.write(s)
    sys.stdout.flush()
    sys.stdout.write("\b" * len(s))

def fmtTime(total):
    hours = (int) (total / 60 / 60)
    minutes = (int) (total / 60 - hours*60)
    seconds = (int) (total - hours*60*60 - minutes*60)
    return '%02d:%02d:%02d' % (hours, minutes, seconds)


if __name__ == '__main__':


    v = ConfigObj(vehicleConfigPath, configspec='Config/vehicle.configspec')
    m = ConfigObj(missionConfigPath % mission, configspec='Config/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    c81File_mainRotor = 'Config/%s'%v['Main Rotor']['AirfoilFile']
    c81File_auxProp = 'Config/%s'%v['Aux Propulsion']['AirfoilFile']
    airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
    airfoildata_auxProp = np.genfromtxt(c81File_auxProp, skip_header=0, skip_footer=0) # read in the airfoil file
    numworkers = multiprocessing.cpu_count()
    workers = [Worker(tasks, results) for i in xrange(numworkers)]
    for w in workers:
        w.start()

    startTime = time.time() - 1 # minus one second so that we don't divide by zero on the first loop
    # keep looping until we actually get our first real result
    goodRows = 0
    totalRows = 0
    outstandingTasks = 0

    for i in xrange(len(ranges)):
        for j in xrange(len(payloads)):
            tasks.put(Task(v, m, airfoildata_mainRotor, airfoildata_auxProp, ranges[i], payloads[j], i, j))
            outstandingTasks += 1
    for i in xrange(numworkers):
        tasks.put(None)

    payloadgrid, rangegrid = np.meshgrid(payloads, ranges)
    weightgrid = np.zeros(rangegrid.shape)
    for t in xrange(outstandingTasks):
        (result, i, j) = results.get()
        showProgress(startTime, time.time(), t, outstandingTasks)
        rangegrid[i][j] = result['Sizing Results']['TotalRange']
        payloadgrid[i][j] = result['Sizing Results']['Payload']
        w = result['Sizing Results']['TotalWeight']
        f = result['Sizing Results']['FuelUsedLb']
        if f>9738:
            weightgrid[i][j] = float('nan')
        else:
            weightgrid[i][j] = result['Sizing Results']['TotalWeight']

    # join/close the tasks queue
    # tasks.join()
    if not halted:
        plt.figure(num=None, figsize=(6, 4), facecolor='w', edgecolor='k')

        # plot.tick_params(labelsize=axislabelfontsize)
        ticks = np.arange(12000, 16000, 1000)
        con = plt.contour(rangegrid*1.852, payloadgrid*0.454, weightgrid*0.454, ticks)
        # cb = plt.colorbar(CS)
        # cb.ax.tick_params(labelsize=axislabelfontsize)
        # CS = plt.contour(baseline[0], baseline[1], baseline[2], GWN, colors='k')
        plt.clabel(con, inline=1, fmt='%1.f kg')

        plt.xlabel('Total Range (km)')
        plt.ylabel('Mission Payload (kg)')
        if mission == 1:
            plt.title('Mission %d' % mission)
        elif mission == 2:
            plt.title('Mission %d' % mission)
        elif mission == 3:
            plt.title('Mission %d' % mission)
        plt.axis([0, 3500, 0, 5000])
        plt.grid(True)
        plt.tight_layout()
        pylab.savefig('Output/Figures/Mission%d_PayloadRange.png' % mission, bbox_inches=0, dpi=600)
        plt.show()


    # print 'Writing out the data!'
    #     for i in xrange(len(output[keys[0]])):
    #         writer.writerow([output[key][i] for key in keys])
