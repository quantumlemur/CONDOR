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




import math
import time
from vehicle import Vehicle
from configobj import ConfigObj

# Script configuration variables here
debug = False
writeOutput = False
vehicleConfigPath = 'Config/vehicle_s92.cfg'
missionConfigPath = 'Config/AHS_mission3.cfg'


def pvar(locals_, vars_):
    s = ['%s: %d' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)


class SizedVehicle:

    def __init__(self, vconfig, mconfig, airfoildata_mainRotor, airfoildata_auxProp):
        self.vconfig = vconfig
        self.mconfig = mconfig
        self.vconfig['Sizing Results']['SizedGrossWeight'] = float('nan')

        self.airfoildata_mainRotor = airfoildata_mainRotor
        self.airfoildata_auxProp = airfoildata_auxProp


    def sizeMission(self):
        """This is the new sizing routine.  It uses bracketing to narrow the tolerances
        of the GW solution until it is found to be within the specified accuracy.  Should
        be totally stable, and doesn't take too many iterations even with wide limits."""
        v = self.vconfig
        m = self.mconfig
        steps = 0
        GWmin = v['Simulation']['GWMin']
        GWmax = v['Simulation']['GWmax']
        viableCandidate = False
        goodAbove = False
        GW = (GWmax + GWmin) / 2
        choppah = Vehicle(v, m, GW, self.airfoildata_mainRotor, self.airfoildata_auxProp) # http://www.youtube.com/watch?v=Xs_OacEq2Sk
        choppah.flyMission()
        while steps<choppah.vconfig['Simulation']['MaxSteps'] and (GWmax-GWmin)>choppah.vconfig['Simulation']['GWTolerance']:
            # Depending on whether we're oversized or undersized for the mission, adjust our GW limits accordingly
            if choppah.vconfig['Sizing Results']['CouldTrim']:
                if choppah.misSize > 0: # we can trim and we're too big
                    GWmax = GW
                    goodAbove = choppah
                    viableCandidate = True
                else: # we can trim and we're too small
                    GWmin = GW
            else: # if we can't trim the current candidate
                if goodAbove: # we can't trim but we could when we were heavier
                    GWmin = GW
                else: # we can't trim and we never could
                    GWmax = GW
            GW = (GWmax - GWmin) / 2 + GWmin
            choppah = Vehicle(v, m, GW, self.airfoildata_mainRotor, self.airfoildata_auxProp)
            choppah.flyMission()
            steps += 1
            #if math.isnan(choppah.misSize) and choppah.vconfig['Weights']['MaxAvailableFuelWeight']<0:
            #    break
            if debug:
                couldTrim = choppah.vconfig['Sizing Results']['CouldTrim']
                couldMission = choppah.misSize > 0
                ms = 99999999999999 if math.isnan(choppah.misSize) else choppah.misSize
                gA = goodAbove is not False
                gAW = goodAbove.vconfig['Sizing Results']['GrossWeight'] if goodAbove else -999999999
                pvar(locals(), ('steps', 'GWmax', 'GWmin', 'couldTrim', 'couldMission'))
        stopReason = ''
        goodRun = False
        if not (choppah.vconfig['Sizing Results']['CouldTrim'] and choppah.misSize>0):
            choppah = goodAbove
        if choppah:
            if not choppah.vconfig['Sizing Results']['CouldTrim']:
                stopReason = 'Cound not trim at all conditions at any mission-capable weight'
            elif choppah.vconfig['Weights']['MaxAvailableFuelWeight'] < 0:
                stopReason = 'Negative calculated max fuel weight'
            elif steps >= choppah.vconfig['Simulation']['MaxSteps']:
                stopReason = 'MaxSteps reached before convergance.  Stopped with bounds: %f  to  %f' % (GWmin, GWmax)
            elif (GWmax-GWmin <= choppah.vconfig['Simulation']['GWTolerance']):
                stopReason = 'Converged to within specified tolerances'
                goodRun = True
            else:
                stopReason = 'Stopped with some other reason'
            choppah.vconfig['Sizing Results']['StopReason'] = stopReason
            choppah.vconfig['Sizing Results']['GoodRun'] = goodRun
            if goodRun:
                choppah.vconfig['Sizing Results']['Optimized'] = True
                choppah.vconfig['Sizing Results']['SizedGrossWeight'] = GW
            else:
                choppah.vconfig['Sizing Results']['Optimized'] = False
                choppah.vconfig['Sizing Results']['SizedGrossWeight'] = float('nan')
            if debug: print('Optimized: %s     %s' % (goodRun, stopReason))
            if writeOutput: choppah.write()
        return choppah

if __name__ == '__main__':
    debug = True
    from time import clock
    from configobj import ConfigObj
    from validate import Validator
    import numpy as np
    v = ConfigObj(vehicleConfigPath, configspec='Config/vehicle.configspec')
    m = ConfigObj(missionConfigPath, configspec='Config/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    startTime = clock()
    c81File_mainRotor = 'Config/%s'%v['Main Rotor']['AirfoilFile']
    c81File_auxProp = 'Config/%s'%v['Aux Propulsion']['AirfoilFile']
    airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
    airfoildata_auxProp = np.genfromtxt(c81File_auxProp, skip_header=0, skip_footer=0) # read in the airfoil file
    blah = SizedVehicle(v, m, airfoildata_mainRotor, airfoildata_auxProp)
    veh = blah.sizeMission()
    if veh: veh.write()
    stopTime = clock()
    elapsed = stopTime - startTime
    if debug: print('Elapsed time: %f' % elapsed)