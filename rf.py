import ConfigParser


from vehicle import Vehicle


debug = True
writeOutput = True

def pvar(locals_, vars_):
    s = ['%s: %d' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)


class SizedVehicle:
    
    def __init__(self, vconfig, mconfig):
        self.vconfig = vconfig
        self.mconfig = mconfig

    
    def sizeMission(self):
        """This is the new sizing routine.  It uses bracketing to narrow the tolerances
        of the GW solution until it is found to be within the specified accuracy.  Should
        be totally stable, and doesn't take too many iterations even with wide limits."""
        v = self.vconfig
        m = self.mconfig
        steps = 0
        GWmin = v['Simulation']['GWMin']
        GWmax = v['Simulation']['GWmax']
        GW = (GWmax - GWmin) / 2 + GWmin
        choppah = Vehicle(v, m, GW) # http://www.youtube.com/watch?v=Xs_OacEq2Sk
        choppah.flyMission()
        while GWmax-GWmin>v['Simulation']['GWTolerance'] and steps<v['Simulation']['MaxSteps']:
            if (not choppah.vconfig['Sizing Results']['CouldTrim']) or choppah.misSize > 0: # If we can't trim it, treat it as if it's too heavy
                GWmax = GW
            else:
                GWmin = GW
            GW = (GWmax - GWmin) / 2 + GWmin
            choppah = Vehicle(v, m, GW)
            choppah.flyMission()
            steps += 1
            if debug:
                EWf = choppah.vconfig['Weights']['EmptyWeightFraction']
                pvar(locals(), ('steps', 'GWmax', 'GW', 'GWmin'))
        stopReason = ''
        goodRun = False
        if not choppah.vconfig['Sizing Results']['CouldTrim']:
            stopReason = 'Cound not trim at all desired conditions'
        elif steps >= v['Simulation']['MaxSteps']:
            stopReason = 'MaxSteps reached before convergance.  Stopped with bounds: %f  to  %f' % (GWmin, GWmax)
        elif GWmax-GWmin <= v['Simulation']['GWTolerance']:
            stopReason = 'Converged to within specified tolerances'
            goodRun = True
        else:
            stopReason = 'You should never see this text'
        #choppah.generatePowerCurve()
        #choppah.findHoverCeiling()
        v['Simulation']['StopReason'] = stopReason
        v['Simulation']['GoodRun'] = goodRun
        v['Sizing Results']['SizedGrossWeight'] = GW
        if goodRun:
            v['Sizing Results']['Optimized'] = True
        else:
            v['Sizing Results']['Optimized'] = False
        if debug: print('Optimized: %s     %s' % (goodRun, stopReason))
        if writeOutput: choppah.write()

if __name__ == '__main__':
    from time import clock
    from configobj import ConfigObj
    from validate import Validator
    startTime = clock()
    v = ConfigObj('Config/vehicle.cfg', configspec='Config/vehicle.configspec')
    m = ConfigObj('Config/mission.cfg', configspec='Config/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    blah = SizedVehicle(v, m)
    blah.sizeMission()
    stopTime = clock()
    elapsed = stopTime - startTime
    if debug: print('Elapsed time: %f' % elapsed)