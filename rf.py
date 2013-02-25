import math
import time
from vehicle import Vehicle
from configobj import ConfigObj


debug = False
writeOutput = False

def pvar(locals_, vars_):
    s = ['%s: %d' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)


class SizedVehicle:
    
    def __init__(self, vconfig, mconfig, airfoildata):
        self.vconfig = vconfig
        self.mconfig = mconfig
        self.vconfig['Sizing Results']['SizedGrossWeight'] = float('nan')

        self.airfoildata = airfoildata 

    
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
        GW = (GWmax - GWmin) / 2 + GWmin
        choppah = Vehicle(v, m, GW, self.airfoildata) # http://www.youtube.com/watch?v=Xs_OacEq2Sk
        choppah.flyMission()
        while not (GWmax-GWmin<choppah.vconfig['Simulation']['GWTolerance']) and steps<choppah.vconfig['Simulation']['MaxSteps']:
            # Depending on whether we're oversized or undersized for the mission, adjust our GW limits accordingly
            if choppah.misSize < 0: # lower the max if we're either too heavy to trim or if we're trimmed but oversized for the mission
                # here we are undersized and trimmable, and will increase the GW minimum
                GWmin = GW
            else:
                # here we are oversized or untrimmable, and will decrease the GW maximum
                GWmax = GW
                if choppah.vconfig['Sizing Results']['CouldTrim']: # if we're oversized but trimmable, then we know we have a viable candidate
                    viableCandidate = True
            GW = (GWmax - GWmin) / 2 + GWmin
            choppah = Vehicle(v, m, GW, self.airfoildata)
            choppah.flyMission()
            steps += 1
            if debug:
                couldTrim = choppah.vconfig['Sizing Results']['CouldTrim']
                couldMission = choppah.misSize > 0
                ms = 0 if math.isnan(choppah.misSize) else choppah.misSize
                pvar(locals(), ('steps', 'GWmax', 'GW', 'GWmin', 'couldTrim', 'couldMission', 'ms', 'viableCandidate'))
        stopReason = ''
        goodRun = False
        if not viableCandidate:
            stopReason = 'Cound not trim at all conditions at any mission-capable weight'
        elif steps >= choppah.vconfig['Simulation']['MaxSteps']:
            stopReason = 'MaxSteps reached before convergance.  Stopped with bounds: %f  to  %f' % (GWmin, GWmax)
        elif (GWmax-GWmin <= choppah.vconfig['Simulation']['GWTolerance']) and choppah.vconfig['Sizing Results']['CouldTrim']:
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
        self.vconfig = choppah.vconfig
        self.mconfig = choppah.mconfig
        return choppah

if __name__ == '__main__':
    debug = True
    from time import clock
    from configobj import ConfigObj
    from validate import Validator
    startTime = clock()
    v = ConfigObj('Config/vehicle.cfg', configspec='Config/vehicle.configspec')
    m = ConfigObj('Config/mission_singlesegment.cfg', configspec='Config/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    blah = SizedVehicle(v, m)
    veh = blah.sizeMission()
    stopTime = clock()
    elapsed = stopTime - startTime
    if debug: print('Elapsed time: %f' % elapsed)