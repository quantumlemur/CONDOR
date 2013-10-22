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



import matplotlib
matplotlib.use('Agg')


import sys
import math
import numpy as np
from time import time
import scipy.ndimage
#import matplotlib.pylab as pylab
import matplotlib.pyplot as plt
import multiprocessing
from mpl_toolkits.axes_grid1 import make_axes_locatable


from rf import SizedVehicle
from vehicle import Vehicle
from configobj import ConfigObj
from validate import Validator


def pvar(locals_, vars_):
    s = ['%s: %d' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)

# BEGIN SCRIPT CONFIG BLOCK
debug = False


DiskLoadingrange = [4, 15]
Solidityrange = [0.05, 0.15]
TipSpeedrange = [400, 800]
GWrange = [15000, 45000]

numPerParam = 20
sweeplimits = [[100, 250], [300, 1000]] # speed, range
baselineRange = 553.
baselineSpeed = 153.

# plot visualization settings
singlefigW = 3.4
singlefigH = 2.5
doublefigW = 6.5
doublefigH = 4.5
figDPI = 300
inline_spacing = 5
contourfontsize = 6
titlefontsize = 8
labelfontsize = 8
labelboxcolor = 'gray'
labeltextcolor = ''
labelalpha = .8
labelpad = 5
labelHoffset = -20
labelVoffset = 50
singleHoffset = 5
singleVoffset = 20
badcolor = '#bb1111'
badmarker = 'ro'
goodcolor = '#22ee22'
goodmarker = 'go'
pointmarkersize = 8
pointfontsize = 8
axislabelfontsize = 8
GWtickspacing = 4000
GWticklowpadding = 4000
GWtickhighpadding = 4000
MRPtickspacing = 250
tickresolution = 1000

# plot display settings
splitFourPlots = True
displayPlots = False
saveData = False
saveFigures = True
annotatePlots = True
showTitle = True
smoothPlots = False

# script behavior settings plot selection
generateNewBaseline = False
generateNewData = False

plotScalingPlots = False
plotPerformanceCurve = False

plotRangeSpeedContour = False
plotWeightImprovements = False
plotDragImprovements = False
plotSFCImprovements = False
plotAllImprovements = True
plotSweepContours = [] # ['Solidity', 'DiskLoading', 'TipSpeed', 'SpanRadiusRatio']
plotExtraContours = False

# configs
vehicleConfigPath = 'Config/vehicle_s92.cfg'
missionConfigPath = 'Config/mission_singlesegment.cfg'
# END SCRIPT CONFIG BLOCK


vconfig = ConfigObj(vehicleConfigPath, configspec='Config/vehicle.configspec')
mconfig = ConfigObj(missionConfigPath, configspec='Config/mission.configspec')
vvdt = Validator()
vconfig.validate(vvdt)
mvdt = Validator()
mconfig.validate(mvdt)
c81File_mainRotor = 'Config/%s'%vconfig['Main Rotor']['AirfoilFile']
c81File_auxProp = 'Config/%s'%vconfig['Aux Propulsion']['AirfoilFile']
airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
airfoildata_auxProp = np.genfromtxt(c81File_auxProp, skip_header=0, skip_footer=0) # read in the airfoil file



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
    def __init__(self, i, j, k, vconfig, mconfig, airfoildata_mainRotor, airfoildata_auxProp):
        self.i = i
        self.j = j
        self.k = k
        self.vconfig = vconfig
        self.mconfig = mconfig
        self.airfoildata_mainRotor = airfoildata_mainRotor
        self.airfoildata_auxProp = airfoildata_auxProp
    def __call__(self):
        vconfig = self.vconfig
        mconfig = self.mconfig
        vehicle = SizedVehicle(vconfig, mconfig, self.airfoildata_mainRotor, self.airfoildata_auxProp)
        sizedVehicle = vehicle.sizeMission() # this is now a Vehicle object
        if sizedVehicle:
            #sizedVehicle.generatePowerCurve()
            #sizedVehicle.findHoverCeiling()
            #sizedVehicle.findMaxRange()
            #sizedVehicle.findMaxSpeed()
            v = sizedVehicle.vconfig
        else:
            v = False
        return ((self.i, self.j, self.k), v)

# def sweep(vehicle, num):
#     numPerParam = (int) (num**(1./4))
#     print numPerParam
#     DiskLoadings = np.linspace(DiskLoadingrange[0], DiskLoadingrange[1], numPerParam)
#     Soliditys = np.linspace(Solidityrange[0], Solidityrange[1], numPerParam)
#     TipSpeeds = np.linspace(TipSpeedrange[0], TipSpeedrange[1], numPerParam)
#     GWs = np.linspace(GWrange[0], GWrange[1], numPerParam)


#     output = np.zeros((len(DiskLoadings)*len(Soliditys)*len(TipSpeeds)*len(GWs), 7))
#     i = 0
#     for DiskLoading in DiskLoadings:
#         for Solidity in Soliditys:
#             for TipSpeed in TipSpeeds:
#                 for GW in GWs:
#                     vehicle.vconfig['Main Rotor']['DiskLoading'] = DiskLoading
#                     vehicle.vconfig['Main Rotor']['Solidity'] = Solidity
#                     vehicle.vconfig['Main Rotor']['TipSpeed'] = TipSpeed
#                     vehicle.GW = GW
#                     vehicle.setup()
#                     vehicle.generatePowerCurve()
#                     vehicle.findMaxRange()
#                     output[i] = [GW, DiskLoading, Solidity, TipSpeed, vehicle.SPEEDmaxr, vehicle.SPEEDmaxr, vehicle.maxrange]
#                     i += 1
#     np.savetxt('Output/parameterSweep.csv', output, delimiter=',')

def showProgress(name, startTime, currentTime, currentRow, totalRows):
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
    s = '     %-40s     %3d of %3d      %s of %s     %s remaining %90s' % (name, currentRow, totalRows, fmtTime(elapsedTime), fmtTime(totalTime), fmtTime(remainingTime), bar)
    sys.stdout.write(s)
    sys.stdout.flush()
    sys.stdout.write("\b" * len(s))

def GenerateBaselineData():
    sweepvars = ['Speed', 'Range']
    # pick the spreads of the variables we're going to sweep
    sweepspreads = np.zeros((len(sweepvars), numPerParam))
    for i in range(len(sweepvars)):
        sweepspreads[i] = np.linspace(sweeplimits[i][0], sweeplimits[i][1], numPerParam)
    # initialize the full-factorial array
    output = cartesian(sweepspreads)
    outs = np.zeros((output.shape[0], 1))
    output = np.hstack((output, outs))
    # set up the plotting grids
    SPEED, RANGE, = np.meshgrid(sweepspreads[0], sweepspreads[1])
    GW = np.zeros(RANGE.shape)
    #MCP = np.zeros(RANGE.shape)
    num = output.shape[0]
    tic = time()
    outstandingTasks = 0
    # put tasks in queue
    for i in xrange(GW.shape[0]):
        for j in xrange(GW.shape[1]):
            m = ConfigObj(mconfig)
            v = ConfigObj(vconfig)
            m['Segment 2']['Distance'] = (float) (RANGE[i][j])
            m['Segment 2']['Speed'] = (float) (SPEED[i][j])
            v['Wing']['MaxSpeed'] = (float) (SPEED[i][j])
            tasks.put(Task(i, j, 0, v, m, airfoildata_mainRotor, airfoildata_auxProp))
            outstandingTasks += 1
    # collect finished tasks
    for num in xrange(outstandingTasks):
        showProgress(sys._getframe().f_code.co_name, tic, time(), num, outstandingTasks)
        m = ConfigObj(mconfig)
        v = ConfigObj(vconfig)
        m['Segment 2']['Distance'] = (float) (RANGE[i][j])
        m['Segment 2']['Speed'] = (float) (SPEED[i][j])
        v['Wing']['MaxSpeed'] = (float) (SPEED[i][j])
        (pos, vc) = results.get()
        GW[pos[0]][pos[1]] = vc['Sizing Results']['SizedGrossWeight'] if vc else float('nan')
            #MCP[i][j] = v['Powerplant']['MCP']
    print('')
    #GW[~np.isfinite(GW)] = 0
    # filter data
    # SPEED = scipy.ndimage.zoom(SPEED, 3)
    # RANGE = scipy.ndimage.zoom(RANGE, 3)
    # GW = scipy.ndimage.zoom(GW, 3)
    #MCP = scipy.ndimage.zoom(MCP, 3)

    GWN = np.arange((int)(GW[np.isfinite(GW)].min())/tickresolution*tickresolution-GWticklowpadding, min((int)(GW[np.isfinite(GW)].max()), 50000)/tickresolution*tickresolution+GWtickspacing+GWtickhighpadding, GWtickspacing)

    # find baseline datapoint
    m = ConfigObj(mconfig)
    v = ConfigObj(vconfig)
    m['Segment 2']['Distance'] = baselineRange
    m['Segment 2']['Speed'] = baselineSpeed
    v['Wing']['MaxSpeed'] = baselineSpeed
    vehicle = SizedVehicle(v, m, airfoildata_mainRotor, airfoildata_auxProp)
    vehicle = vehicle.sizeMission()
    if vehicle:
        vehicle.generatePowerCurve()
        vehicle.findHoverCeiling()
        vehicle.findMaxRange()
        vehicle.findMaxSpeed()
        vehicle.write()
        baselineGW = vehicle.vconfig['Sizing Results']['SizedGrossWeight'] if vehicle else float('nan')

    return (SPEED, RANGE, GW, GWN, baselineGW)



def RangeSpeedContour(baseline, filename='Baseline', section=['Main Rotor'], var=['Nothing'], value=[0]):
    filenames = ['WeightImprovements', 'DragImprovements', 'SFCImprovements', 'AllImprovements']
    titles = ['Weight Improvements', 'Drag Improvements', 'SFC Improvements', 'All Improvements']
    sections = [['Weights', 'Weights', 'Weights', 'Weights'], ['Body'], ['Powerplant'], ['Weights', 'Weights', 'Weights', 'Weights', 'Body', 'Powerplant']]
    varss = [['StructureWeightTechImprovementFactor', 'WingWeightTechImprovementFactor', 'EngineWeightTechImprovmentFactor', 'DriveSystemWeightTechImprovementFactor'], ['DragTechImprovementFactor'], ['SFCTechImprovementFactor'], ['StructureWeightTechImprovementFactor', 'WingWeightTechImprovementFactor', 'EngineWeightTechImprovmentFactor', 'DriveSystemWeightTechImprovementFactor', 'DragTechImprovementFactor', 'SFCTechImprovementFactor']]
    values = [[.13, .05, .04, .13], [.23], [.205], [.13, .05, .04, .13, .23, .205]]
    plt.figure(num=None, figsize=(doublefigW, doublefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    for which in xrange(4):
        filename = filenames[which]
        section = sections[which]
        var = varss[which]
        value = values[which]
        title = titles[which]



        sweepvars = ['Speed', 'Range']
        # pick the spreads of the variables we're going to sweep
        sweepspreads = np.zeros((len(sweepvars), numPerParam))
        for i in range(len(sweepvars)):
            sweepspreads[i] = np.linspace(sweeplimits[i][0], sweeplimits[i][1], numPerParam)
        # initialize the full-factorial array
        output = cartesian(sweepspreads)
        outs = np.zeros((output.shape[0], 1))
        output = np.hstack((output, outs))
        # set up the plotting grids
        SPEED, RANGE, = np.meshgrid(sweepspreads[0], sweepspreads[1])
        GW = np.zeros(RANGE.shape)
        #MCP = np.zeros(RANGE.shape)

        num = output.shape[0]
        tic = time()
        if filename == 'Baseline':
            GW = baseline[2]
        else:
            if generateNewData:
                outstandingTasks = 0
                # queue tasks
                for i in xrange(GW.shape[0]):
                    for j in xrange(GW.shape[1]):
                        m = ConfigObj(mconfig)
                        v = ConfigObj(vconfig)
                        m['Segment 2']['Distance'] = (float) (RANGE[i][j])
                        m['Segment 2']['Speed'] = (float) (SPEED[i][j])
                        v['Wing']['MaxSpeed'] = (float) (SPEED[i][j])
                        for k in xrange(len(section)):
                            v[section[k]][var[k]] = value[k]
                        tasks.put(Task(i, j, 0, v, m, airfoildata_mainRotor, airfoildata_auxProp))
                        outstandingTasks += 1
                # collect results back
                for num in xrange(outstandingTasks):
                    showProgress('%s - %s' % (sys._getframe().f_code.co_name, filename), tic, time(), num, outstandingTasks)
                    (pos, vc) = results.get()
                    GW[pos[0]][pos[1]] = vc['Sizing Results']['SizedGrossWeight'] if vc else float('nan')
                    #MCP[i][j] = v['Powerplant']['MCP']
                print('')
            else:
                data = np.load('Output/Data/%sContourData.npz' % filename)
                GW = data['GW']
        # filter data
        # SPEED = scipy.ndimage.zoom(SPEED, 3)
        # RANGE = scipy.ndimage.zoom(RANGE, 3)
        # GW = scipy.ndimage.zoom(GW, 3)
        #MRP = scipy.ndimage.zoom(MRP, 3)

        GWN = baseline[3]

        #MRPN = np.linspace(MRP.min(), MRP.max(), num=1000)

        #plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')

        plot = plt.subplot(2, 2, which)
        plot.tick_params(labelsize=axislabelfontsize)
        if saveData: print 'Saving as Output/Data/%sContourData' % filename
        if saveData: np.savez('Output/Data/%sContourData' % filename, SPEED=SPEED, RANGE=RANGE, GW=GW, GWN=GWN)
        if smoothPlots:
            GW[~np.isfinite(GW)] = 100000
            SPEED = scipy.ndimage.zoom(SPEED, 2)
            RANGE = scipy.ndimage.zoom(RANGE, 2)
            GW = scipy.ndimage.zoom(GW, 2)


        CS = plt.contourf(SPEED, RANGE, GW, GWN)
        CSb = plt.contour(baseline[0], baseline[1], baseline[2], GWN, colors='k')
        plt.clabel(CSb, inline=1, fontsize=contourfontsize, fmt='%1.f')

        plt.xlabel('Ingress Speed (kts)', fontsize=labelfontsize)
        plt.ylabel('Mission Range (nm)', fontsize=labelfontsize)
        if showTitle: plt.title(title, fontsize=titlefontsize)
        plt.grid(True)


        if annotatePlots:
            # measure improvement over baseline
            m = ConfigObj(mconfig)
            v = ConfigObj(vconfig)
            m['Segment 2']['Distance'] = baselineRange
            m['Segment 2']['Speed'] = baselineSpeed
            v['Wing']['MaxSpeed'] = baselineSpeed
            for k in xrange(len(section)):
                v[section[k]][var[k]] = value[k]
            vehicle = SizedVehicle(v, m, airfoildata_mainRotor, airfoildata_auxProp)
            vehicle = vehicle.sizeMission()
            if vehicle:
                newGW = vehicle.vconfig['Sizing Results']['SizedGrossWeight']
                baselineGW = baseline[4]
                percentage = (baselineGW - newGW) / baselineGW
                if percentage<0:
                    s = '{:.1%} GW Increase'.format(-percentage)
                    labeltextcolor = badcolor
                    marker = badmarker
                else:
                    s = '{:.1%} GW Improvement'.format(percentage)
                    labeltextcolor = goodcolor
                    marker = goodmarker
                if abs(percentage) > .001:
                    plt.plot([baselineSpeed], [baselineRange], marker, markersize=pointmarkersize)
                    plt.text(baselineSpeed+singleHoffset, baselineRange+singleVoffset, s, fontweight='bold', fontsize=labelfontsize, color=labeltextcolor, bbox={'facecolor':labelboxcolor, 'edgecolor':labelboxcolor, 'alpha':labelalpha, 'pad':labelpad})

        divider = make_axes_locatable(plt.gca())
        cax = divider.append_axes("right", "5%", pad="3%")
        cb = plt.colorbar(CS, cax=cax)
        cb.ax.tick_params(labelsize=axislabelfontsize)

        #plot = plt.subplot(1, 2, 2)
        #plot.tick_params(labelsize=axislabelfontsize)
        #SPEED = scipy.ndimage.zoom(SPEED, 3)
        #RANGE = scipy.ndimage.zoom(RANGE, 3)
        #MRP = scipy.ndimage.zoom(GW, 3)
        #CS = plt.contourf(SPEED, RANGE, MRP, GWN)
        ##plt.clabel(CS, inline=1, fontsize=10)
        #plt.colorbar(CS)
        #plt.xlabel('Ingress Speed (kts)', fontsize=labelfontsize)
        #plt.ylabel('Ferry Range (nm)', fontsize=labelfontsize)
        #plt.title('MRP contours', fontsize=titlefontsize)
        #plt.grid(True)

    plt.tight_layout()
    if saveFigures: plt.savefig('Output/Figures/%sContour.png' % filename, bbox_inches=0, dpi=figDPI)
    if displayPlots: plt.show()


def SweepContours(baseline):
    for sweepVar in plotSweepContours:
        if sweepVar == 'DiskLoading':
            Spread = [5., 10., 15., 20.]
            section = 'Main Rotor'
            extraContour = 'HoverCeiling'
            title = '%d psf DiskLoading'
        elif sweepVar == 'Solidity':
            Spread = [0.06, 0.08, 0.1, 0.12]
            section = 'Main Rotor'
            extraContour = 'MaxBladeLoadingSeen'
            title = r'$\sigma = %.2f$'
        elif sweepVar == 'TipSpeed':
            Spread = [500., 600., 700., 800.]
            section = 'Main Rotor'
            extraContour = 'HoverCeiling'
            title = '%d fps Tip Speed'
        elif sweepVar == 'SpanRadiusRatio':
            Spread = [0., 1., 2., 3.]
            extraContour = 'Nothing'
            section = 'Wing'
            title = '%.1f Ratio of Wing Span to Rotor Radius'
        elif sweepVar == 'Radius':
            Spread = [25., 30., 35., 40.]
            extraContour = 'Nothing'
            section = 'Main Rotor'
            title = '%d m Radius'

        sweepvars = ['Speed', 'Range']
        # pick the spreads of the variables we're going to sweep
        sweepspreads = np.zeros((len(sweepvars), numPerParam))
        for i in range(len(sweepvars)):
            sweepspreads[i] = np.linspace(sweeplimits[i][0], sweeplimits[i][1], numPerParam)
        # initialize the full-factorial array
        output = cartesian(sweepspreads)
        outs = np.zeros((output.shape[0], 1))
        output = np.hstack((output, outs))
        # set up the plotting grids
        SPEED, RANGE, = np.meshgrid(sweepspreads[0], sweepspreads[1])
        GW = np.zeros((len(Spread), RANGE.shape[0], RANGE.shape[1]))
        MCP = np.zeros((len(Spread), RANGE.shape[0], RANGE.shape[1]))
        extraContours = np.zeros((len(Spread), RANGE.shape[0], RANGE.shape[1]))
        smoothedGW = np.zeros((len(Spread)*3, RANGE.shape[0]*3, RANGE.shape[1]*3))
        smoothedMCP = np.zeros((len(Spread)*3, RANGE.shape[0]*3, RANGE.shape[1]*3))
        totalrows = GW[0].shape[0]*len(Spread)
        rowcount = 0
        tic = time()
        if generateNewData:
            outstandingTasks = 0
            for DLi in xrange(len(Spread)):
                for i in xrange(GW[0].shape[0]):
                    for j in xrange(GW[0].shape[1]):
                        m = ConfigObj(mconfig)
                        v = ConfigObj(vconfig)
                        m['Segment 2']['Distance'] = (float) (RANGE[i][j])
                        m['Segment 2']['Speed'] = (float) (SPEED[i][j])
                        v['Wing']['MaxSpeed'] = (float) (SPEED[i][j])
                        v[section][sweepVar] = Spread[DLi]
                        v['Main Rotor']['SlowedTipSpeed'] = v['Main Rotor']['TipSpeed']
                        tasks.put(Task(i, j, DLi, v, m, airfoildata_mainRotor, airfoildata_auxProp))
                        outstandingTasks += 1
            for num in xrange(outstandingTasks):
                if i>0: showProgress('%s - %s' % (sys._getframe().f_code.co_name, sweepVar), tic, time(), num, outstandingTasks)
                (pos, vc) = results.get()
                GW[pos[2]][pos[0]][pos[1]] = vc['Sizing Results']['SizedGrossWeight'] if vc else float('nan')
                MCP[pos[2]][pos[0]][pos[1]] = vc['Powerplant']['MCP'] if vc else float('nan')
                # extraContours[pos[2]][pos[0]][pos[1]] = vc['Performance'][extraContour] if vc else float('nan')
                # smoothedGW[DLi] = scipy.ndimage.zoom(GW[DLi], 3)
                # smoothedMCP[DLi] = scipy.ndimage.zoom(GW[DLi], 3)

            print('')
        else:
            data = np.load('Output/Data/%sContourData.npz' % sweepVar)
            GW = data['GW']
            MCP = data['MCP']
            extraContours = data['extraContours']

        # SPEED = scipy.ndimage.zoom(SPEED, 3)
        # RANGE = scipy.ndimage.zoom(RANGE, 3)
        # MCP = smoothedMCP
        # GW = smoothedGW
        # extraContours = scipy.ndimage.zoom(extraContours, 3)

        GWN = baseline[3]


        if saveData: print 'Saving as Output/Data/%sContourData' % sweepVar
        if saveData: np.savez('Output/Data/%sContourData' % sweepVar, SPEED=SPEED, RANGE=RANGE, GW=GW, GWN=GWN, MCP=MCP, extraContours=extraContours)
        if smoothPlots:
            GW[~np.isfinite(GW)] = 999999999
            SPEED = scipy.ndimage.zoom(SPEED, 3)
            RANGE = scipy.ndimage.zoom(RANGE, 3)
            GW = scipy.ndimage.zoom(GW, 3)
            extraContours = scipy.ndimage.zoom(extraContours, 3)


        if not splitFourPlots:
            plt.figure(num=None, figsize=(doublefigW, doublefigH), dpi=figDPI, facecolor='w', edgecolor='k')

        for DLi in xrange(len(Spread)):
            if splitFourPlots:
                plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
                plot = plt.subplot(1, 1, 1)
            else:
                plot = plt.subplot(2, 2, DLi+1)
            plot.tick_params(labelsize=axislabelfontsize)
            # try:
            CS = plt.contourf(SPEED, RANGE, GW[DLi], GWN)
            CSb = plt.contour(baseline[0], baseline[1], baseline[2], GWN, colors='k')
            plt.clabel(CSb, inline=1, fontsize=contourfontsize, fmt='%1.f')
            plt.xlabel('Ingress Speed (kts)', fontsize=labelfontsize)
            plt.ylabel('Mission Range (nm)', fontsize=labelfontsize)
            if showTitle or not splitFourPlots: plt.title(title % Spread[DLi], fontsize=titlefontsize)
            plt.grid(True)
            if plotExtraContours and extraContour != 'Nothing':
                CS = plt.contour(SPEED, RANGE, extraContours[DLi], colors='r')
                plt.clabel(CS, inline=1, fontsize=contourfontsize, fmt='%.4f')
            if annotatePlots:
                # measure improvement over baseline
                m = ConfigObj(mconfig)
                v = ConfigObj(vconfig)
                m['Segment 2']['Distance'] = baselineRange
                m['Segment 2']['Speed'] = baselineSpeed
                v['Wing']['MaxSpeed'] = baselineSpeed
                v[section][sweepVar] = Spread[DLi]
                vehicle = SizedVehicle(v, m, airfoildata_mainRotor, airfoildata_auxProp)
                vehicle = vehicle.sizeMission()
                if vehicle:
                    newGW = vehicle.vconfig['Sizing Results']['SizedGrossWeight']
                    baselineGW = baseline[4]
                    percentage = (baselineGW - newGW) / baselineGW
                    if percentage<0:
                        s = '{:.1%} GW Increase'.format(-percentage)
                        labeltextcolor = badcolor
                        marker = badmarker
                    else:
                        s = '{:.1%} GW Improvement'.format(percentage)
                        labeltextcolor = goodcolor
                        marker = goodmarker
                    if abs(percentage) > .001:
                        plt.plot([baselineSpeed], [baselineRange], marker, markersize=pointmarkersize)
                        plt.text(baselineSpeed+labelHoffset, baselineRange+labelVoffset, s, fontweight='bold', fontsize=labelfontsize, color=labeltextcolor, bbox={'facecolor':labelboxcolor, 'edgecolor':labelboxcolor, 'alpha':labelalpha, 'pad':labelpad})
            divider = make_axes_locatable(plt.gca())
            cax = divider.append_axes("right", "5%", pad="3%")
            cb = plt.colorbar(CS, cax=cax)
            cb.ax.tick_params(labelsize=axislabelfontsize)
            if splitFourPlots:
                plt.tight_layout()
                if plotExtraContours:
                    sV = '%sExtra' % sweepVar
                else:
                    sV = sweepVar
                if saveFigures: plt.savefig('Output/Figures/%s_%f_%d_GWContour.png' % (sV, Spread[DLi], DLi), bbox_inches=0, dpi=figDPI)
                if displayPlots: plt.show()
            # except:
            #     print GW[DLi]
            #     print GWN
        if not splitFourPlots:
            plt.tight_layout()
            if plotExtraContours:
                sV = '%sExtra' % sweepVar
            else:
                sV = sweepVar
            if saveFigures: plt.savefig('Output/Figures/%sGWContour.png' % sV, bbox_inches=0, dpi=figDPI)
            if displayPlots: plt.show()

def PerformanceCurve():
    v = ConfigObj(vconfig)
    m = ConfigObj(mconfig)
    choppah = Vehicle(v, m, 25000, airfoildata_mainRotor, airfoildata_auxProp)
    choppah.generatePowerCurve()
    #choppah.write()

    plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    plt.plot(choppah.vconfig['Power Curve']['Speeds'], choppah.vconfig['Power Curve']['PowersSL'])
    plt.plot(choppah.vconfig['Power Curve']['Speeds'], choppah.vconfig['Power Curve']['PowersCruise'])
    MCPspeeds = [0, 200]
    MCPSL = [2043*2, 2043*2]
    MCPalt = [3104, 3104]
    plt.plot(MCPspeeds, MCPSL, 'b')
    plt.plot(MCPspeeds, MCPalt, 'g')
    plt.axis([0, 200, 1000, 6000])
    plt.legend(('Sea Level', '14k ft'), fontsize=labelfontsize)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xlabel('Airspeed (kts)', fontsize=labelfontsize)
    plt.ylabel('Power (hp)', fontsize=labelfontsize)
    plt.title('Predicted S-92 Power Curve', fontsize=titlefontsize)
    plt.grid(True)

    if saveFigures: plt.savefig('Output/Figures/S92PerformanceCurve.png', bbox_inches=0, dpi=figDPI)
    if displayPlots: plt.show()





def ScalingPlots():
    # flat plate drag area
    plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    dragGW = np.arange(0, 40000, 100)
    flatplatedrag = 0.25 * dragGW**.5
    heliDrags = np.array([15, 21, 22.5, 37, 32, 38, 41, 43, 56, 40])
    heliGWs = np.array([2.2, 8, 9, 13, 18, 20.5, 22, 33, 39.5, 25]) * 1000
    heliLabels = np.array(['S-52', 'S-55', 'UH-1', 'S-58', 'S-61', 'CH-46', 'SA 321', 'CH-47', 'CH-53A', 'S-92'])
    for i in xrange(len(heliLabels)):
        plt.text(heliGWs[i]-1500, heliDrags[i]+1, heliLabels[i], fontsize=labelfontsize)#, bbox={'facecolor':labelboxcolor, 'edgecolor':labelboxcolor, 'alpha':labelalpha, 'pad':labelpad})
    plt.plot(heliGWs, heliDrags, 'ko')
    plot = plt.plot(dragGW, flatplatedrag)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xlabel('Gross Weight (lbs)', fontsize=labelfontsize)
    plt.ylabel('Flat Plate Drag Area (sq ft)', fontsize=labelfontsize)
    plt.title('Flat Plate Drag Scaling', fontsize=titlefontsize)
    plt.tight_layout()
    if saveFigures: plt.savefig('Output/Figures/flatPlateDragScaling.png', bbox_inches=0, dpi=figDPI)
    if displayPlots: plt.show()

    # engine weight
    plt.figure(num=None, figsize=(doublefigW, doublefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    plt.subplot(2, 2, 1)
    MCP = np.arange(0, 4000, 10)
    weight = ((0.1054*(MCP)**2+358*(MCP)+2.757*10**4)/((MCP)+1180))
    plot = plt.plot(MCP, weight)
    plt.text(200, 580, r'$\frac{0.1054*MCP^2 + 358*MCP + 2.757*10^4}{MCP+1180}$', fontsize=labelfontsize+4)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xticks(np.arange(0, 5000, 1000))
    plt.xlabel('Max Continuous Power (hp)', fontsize=labelfontsize)
    plt.ylabel('Engine Weight (lbs)', fontsize=labelfontsize)
    plt.title('Engine Weight Scaling', fontsize=titlefontsize)
    plt.tight_layout()

    # drive system weight
    plt.subplot(2, 2, 2)
    GW = np.arange(0, 40000, 100)
    MCP = [2000., 4000., 6000.]
    weight = np.zeros((len(MCP), GW.size))
    DL = 10.
    for i in range(len(MCP)):
        weight[i] = (525.*(GW/1000.)**1.14)/((GW/MCP[i])**0.763*DL**0.381)
        plot = plt.plot(GW, weight[i])
        plt.text(30000, weight[i][300]-200, '%d hp' % MCP[i], fontsize=labelfontsize)
    plt.text(15000, 400, r'$\frac{525*(GW/1000)^{1.14}}{(GW/MCP)^{.763}*DL^{.381}}$', fontsize=labelfontsize+5)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xticks(np.arange(0, 50000, 10000))
    plt.xlabel('Gross Weight (lbs)', fontsize=labelfontsize)
    plt.ylabel('Drive System Weight (lbs)', fontsize=labelfontsize)
    plt.title('Drive System Scaling (DL=10psf)', fontsize=titlefontsize)
    plt.tight_layout()

    # wing weight
    plt.subplot(2, 2, 3)
    GW = np.arange(0, 40000, 100)
    SRR = [1., 2., 3.]
    weights = np.zeros((len(SRR), GW.size))
    DL = 10
    for i in xrange(len(SRR)):
        weights[i] = 0.00272*GW**1.4/DL**0.8*SRR[i]**0.8
        plt.plot(GW, weights[i])
        plt.text(30000, weights[i][300]-100, r'$\frac{Span}{R}=%d$' % SRR[i], fontsize=labelfontsize)
    plt.text(5000, 2250, r'$\frac{.00272*GW^{1.4}}{DL^{.8}}*\frac{Span}{R}^{.8}$', fontsize=labelfontsize+5)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xticks(np.arange(0, 50000, 10000))
    plt.xlabel('Gross Weight (lbs)', fontsize=labelfontsize)
    plt.ylabel('Wing Weight (lbs)', fontsize=labelfontsize)
    plt.title('Wing Weight Scaling (DL=10psf)', fontsize=titlefontsize)
    plt.tight_layout()

    # SFC
    plt.subplot(2, 2, 4)
    gamma = np.arange(.5, 4, .1)
    SFCfrac = (-0.00932*gamma**2+0.865*gamma+0.445)/(gamma+0.301)
    plt.plot(gamma, SFCfrac)
    plt.text(.8, 1.05, r'$\frac{SFC}{SFC_{baseline}}=\frac{-.00932*\gamma^2+.865*\gamma+.445}{\gamma+.301}$', fontsize=labelfontsize+4)
    plt.tick_params(labelsize=axislabelfontsize)
    #plt.xticks(np.arange(0, 50000, 10000))
    plt.xlabel(r'$\gamma = MCP/MCP_{baseline}$', fontsize=labelfontsize)
    plt.ylabel(r'$SFC/SFC_{baseline}$', fontsize=labelfontsize)
    plt.title('Engine SFC Scaling', fontsize=titlefontsize)
    plt.tight_layout()

    if displayPlots: plt.show()
    if saveFigures: plt.savefig('Output/Figures/WeightsAndSFCScaling.png', bbox_inches=0, dpi=figDPI)


# def SolidityContour(vconfig, mconfig, num, baseline):
#     sweepvars = ['Speed', 'Range']
#     Spread = [0.06, 0.08, 0.1, 0.12]
#     numPerParam = (int) (num)
#     # pick the spreads of the variables we're going to sweep
#     sweepspreads = np.zeros((len(sweepvars), numPerParam))
#     for i in range(len(sweepvars)):
#         sweepspreads[i] = np.linspace(sweeplimits[i][0], sweeplimits[i][1], numPerParam)
#     # initialize the full-factorial array
#     output = cartesian(sweepspreads)
#     outs = np.zeros((output.shape[0], 1))
#     output = np.hstack((output, outs))
#     # set up the plotting grids
#     SPEED, RANGE, = np.meshgrid(sweepspreads[0], sweepspreads[1])
#     GW = np.zeros((len(Spread), RANGE.shape[0], RANGE.shape[1]))
#     MRP = np.zeros((len(Spread), RANGE.shape[0], RANGE.shape[1]))
#     totalrows = GW[0].shape[0]*len(Spread)
#     rowcount = 0
#     tic = clock()
#     for DLi in xrange(len(Spread)):
#         for i in xrange(GW[0].shape[0]):
#             rowcount += 1
#             if i>0: showProgress(sys._getframe().f_code.co_name, tic, clock(), rowcount, totalrows)
#             for j in xrange(GW[0].shape[1]):
#                 mconfig['Segment 2']['Distance'] = (float) (RANGE[i][j])
#                 mconfig['Segment 2']['Speed'] = (float) (SPEED[i][j])
#                 vconfig['Main Rotor']['Solidity'] = Spread[DLi]
#                 vehicle = SizedVehicle(vconfig, mconfig, airfoildata_mainRotor, airfoildata_auxProp)
#                 vehicle = vehicle.sizeMission()
#                 GW[DLi][i][j] = vehicle.vconfig['SizedGrossWeight']
#                 MRP[DLi][i][j] = vehicle.vconfig['Powerplant']['MCP']
#     print('')
#     plt.figure(num=None, figsize=(figW, figH), dpi=figDPI, facecolor='w', edgecolor='k')
#     GWN = np.arange((int)(GW[np.isfinite(GW)].min()/1000)*1000, (int)((GW[np.isfinite(GW)].max()/1000)+1)*1000, 2000)
#     MRPN = np.arange((int)(MRP[np.isfinite(MRP)].min()/250)*250, (int)((MRP[np.isfinite(MRP)].max()/250)+1)*250, 250)
#     for DLi in xrange(len(Spread)):
#         plot = plt.subplot(2, 2, DLi+1)
#         plot.tick_params(labelsize=axislabelfontsize)
#         CS = plt.contourf(SPEED, RANGE, GW[DLi], GWN)
#         #CS = plt.contourf(SPEED, RANGE, MRP[DLi], MRPN)
#         #plt.clabel(CS, inline=1, inline_spacing=inline_spacing, fontsize=contourfontsize)
#         plt.colorbar(CS)
#         plt.xlabel('Ingress Speed (kts)', fontsize=labelfontsize)
#         plt.ylabel('Mission Range (nm)', fontsize=labelfontsize)
#         plt.title(r'$\sigma = %.2f$' % Spread[DLi], fontsize=titlefontsize)
#         plt.grid(True)
#     plt.tight_layout()
#     pylab.savefig('Output/SolidityGWContour.png', bbox_inches=0)
#     if displayPlots: plt.show()

#     #plt.figure(num=None, figsize=(figW, figH), dpi=figDPI, facecolor='w', edgecolor='k')
#     #for DLi in xrange(len(Spread)):
#     #    plot = plt.subplot(2, 2, DLi+1)
#     #    plot.tick_params(labelsize=axislabelfontsize)
#     #    #CS = plt.contourf(SPEED, RANGE, GW[DLi], GWN)
#     #    CS = plt.contourf(SPEED, RANGE, MRP[DLi], MRPN)
#     #    #plt.clabel(CS, inline=1, inline_spacing=inline_spacing, fontsize=contourfontsize)
#     #    plt.colorbar(CS)
#     #    plt.xlabel('Ingress Speed (kts)', fontsize=labelfontsize)
#     #    plt.ylabel('Ferry Range (nm)', fontsize=labelfontsize)
#     #    plt.title('%d psf DiskLoading' % Spread[DLi], fontsize=titlefontsize)
#     #    plt.grid(True)
#     #plt.tight_layout()
#     #pylab.savefig('Output/SolidityMCPContour.png', bbox_inches=0)
#     if displayPlots: plt.show()
#     if saveData: np.savez('Output/Data/SolidityContourData', SPEED=SPEED, RANGE=RANGE, GW=GW, GWN=GWN, MRP=MRP, MRPN=MRPN)

def fmtTime(total):
    hours = (int) (total / 60 / 60)
    minutes = (int) (total / 60 - hours*60)
    seconds = (int) (total - hours*60*60 - minutes*60)
    return '%02d:%02d:%02d' % (hours, minutes, seconds)

# def RFsweep(vehicle, num):
#     sweepvars = ['Range', 'Speed']
#     numPerParam = (int) (num**(1./len(sweepvars)))
#     # pick the spreads of the variables we're going to sweep
#     sweepspreads = np.zeros((len(sweepvars), numPerParam))
#     for i in range(len(sweepvars)):
#         sweepspreads[i] = np.linspace(sweeplimits[i][0], sweeplimits[i][1], numPerParam)
#     # initialize the full-factorial array
#     output = cartesian(sweepspreads)
#     outs = np.zeros((output.shape[0], 1))
#     output = np.hstack((output, outs))

#     num = output.shape[0]
#     starttime = clock()
#     tic = clock()
#     toc = clock()
#     for i in xrange(num):
#         if i%100 == 0:
#             tic = toc
#             toc = time()
#             elapsed = (toc - tic) / 60
#             timePerRow = elapsed / 100
#             remainingRows = num - i
#             remainingTime = remainingRows * timePerRow
#             totalTime = num * timePerRow
#             print '%d of %d      Time: %.1fm of %.1fm.  %.1fm remaining' % (i, num, (toc-starttime)/60, totalTime, remainingTime)
#         vehicle.mconfig['Segment 1']['Distance'] = (float) (output[i][0])
#         vehicle.mconfig['Segment 1']['Speed'] = (float) (output[i][1])
#         vehicle.setMission(vehicle.mconfig)
#         vehicle.sizeMission()
#         output[i][2] = vehicle.vconfig['SizedGrossWeight']

#     np.savetxt('Output/parameterSweep.csv', output, delimiter=',')



def cartesian(arrays, out=None):
    """
    Generate a cartesian product of input arrays.
    """

    arrays = [np.asarray(x) for x in arrays]
    dtype = arrays[0].dtype

    n = np.prod([x.size for x in arrays])
    if out is None:
        out = np.zeros([n, len(arrays)], dtype=dtype)

    m = n / arrays[0].size
    out[:,0] = np.repeat(arrays[0], m)
    if arrays[1:]:
        cartesian(arrays[1:], out=out[0:m,1:])
        for j in xrange(1, arrays[0].size):
            out[j*m:(j+1)*m,1:] = out[0:m,1:]
    return out




    #output = np.zeros((len(DiskLoadings)*len(Soliditys)*len(TipSpeeds)*len(GWs), 7))
    #i = 0
    #for DiskLoading in DiskLoadings:
    #    for Solidity in Soliditys:
    #        for TipSpeed in TipSpeeds:
    #            for GW in GWs:
    #                #vehicle.vconfig['Main Rotor']['DiskLoading'] = DiskLoading
    #                #vehicle.vconfig['Main Rotor']['Solidity'] = Solidity
    #                #vehicle.vconfig['Main Rotor']['TipSpeed'] = TipSpeed
    #                #vehicle.GW = GW
    #                vehicle.setup()
    #                blah.setMission(m)
    #                vehicle.sizeMission2()
    #
    #                output[i] = [GW, DiskLoading, Solidity, TipSpeed, vehicle.SPEEDmaxr, vehicle.SPEEDmaxr, vehicle.maxrange]
    #                i += 1
    #np.savetxt('Output/parameterSweep.csv', output, delimiter=',')


if __name__ == '__main__':


    startTime = time()

    tasks = multiprocessing.JoinableQueue()
    results = multiprocessing.Queue()
    numworkers = multiprocessing.cpu_count() * 2
    workers = [Worker(tasks, results) for i in xrange(numworkers)]
    for w in workers:
        w.start()

    #blah = SizedVehicle(v, m)
    if generateNewBaseline:
        baseline = GenerateBaselineData()
        np.save('Output/Data/baseline.npy', baseline)
    else:
        baseline = np.load('Output/Data/baseline.npy')
    if plotScalingPlots: ScalingPlots()
    if plotPerformanceCurve: PerformanceCurve()
    if plotRangeSpeedContour: RangeSpeedContour(baseline)
    if plotWeightImprovements: RangeSpeedContour(baseline, 'WeightImprovements', ['Weights', 'Weights', 'Weights', 'Weights'], ['StructureWeightTechImprovementFactor', 'WingWeightTechImprovementFactor', 'EngineWeightTechImprovmentFactor', 'DriveSystemWeightTechImprovementFactor'], [.13, .05, .04, .13])
    if plotDragImprovements: RangeSpeedContour(baseline, 'DragImprovements', ['Body'], ['DragTechImprovementFactor'], [.23])
    if plotSFCImprovements: RangeSpeedContour(baseline, 'SFCImprovements', ['Powerplant'], ['SFCTechImprovementFactor'], [.205])
    if plotAllImprovements: RangeSpeedContour(baseline, 'AllImprovements', ['Weights', 'Weights', 'Weights', 'Weights', 'Body', 'Powerplant'], ['StructureWeightTechImprovementFactor', 'WingWeightTechImprovementFactor', 'EngineWeightTechImprovmentFactor', 'DriveSystemWeightTechImprovementFactor', 'DragTechImprovementFactor', 'SFCTechImprovementFactor'], [.13, .05, .04, .13, .23, .205])
    SweepContours(baseline)

    # shut down the workers
    for i in xrange(numworkers):
        tasks.put(None)
    tasks.join()

    stopTime = time()
    elapsed = stopTime - startTime
    if debug: print('Elapsed time: %f' % elapsed)
