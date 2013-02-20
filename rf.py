import ConfigParser


from vehicle import Vehicle


debug = True
writeOutput = False

def pvar(locals_, vars_):
    s = ['%s: %d' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)


class SizedVehicle:
    
    def __init__(self, vconfig, mconfig):
        self.vconfig = vconfig
        self.mconfig = mconfig
        #self.setMission(mconfig)
    
    def setMission(self, mconfig):
        # This function sets the mission, and analyzes all the mission segments, fleshing them out with some calculated values
        self.mconfig = mconfig
        m = self.mconfig
        v = self.vconfig
        totaltime = 0
        segment = 1
        numsegs = 0
        while 'Segment %s' % segment in m:
            seg = 'Segment %s' % segment
            m[seg]['Time'] = m[seg]['Distance'] / m[seg]['Speed'] * 60 # time in minutes
            m[seg]['StartTime'] = totaltime
            totaltime += m[seg]['Time']
            m[seg]['EndTime'] = totaltime
            m[seg]['DensityRatio'] = m[seg]['Density'] / 0.002377
            m[seg]['Kl'] = 1 - m[seg]['Speed']/(1.3*v['Wing']['MaxSpeed'])
            m[seg]['AdvanceRatio'] = m[seg]['Speed']*1.68781 / v['Main Rotor']['TipSpeed']
            m[seg]['Kmu'] = 1 + 3*m[seg]['AdvanceRatio']**2 + 5*m[seg]['AdvanceRatio']**4
            segment += 1
            numsegs += 1
        m['TotalTime'] = totaltime
        m['NumSegments'] = numsegs
        # Step through the segments backwards and find the payload delta from the next segment
        prevLoad = 0
        for i in range(m['NumSegments'], 0, -1):
            seg = 'Segment %s' % i
            load = m[seg]['CrewWeight'] + m[seg]['PayloadWeight'] + m[seg]['MiscWeight']
            m[seg]['DeltaLoad'] = load - prevLoad
            prevLoad = load
    
    def sizeMissionOld(self):
        """This is the old sizing routine.  It uses the approximate missize amount
        given by the fuel surplus/deficiency to adjust the GW.  Seems to be stable
        but I don't know for sure.  Not updated any more."""
        v = self.vconfig
        m = self.mconfig
        steps = 0
        missize = 99999
        GW = v['Simulation']['StartGW']
        while abs(missize)>v['Simulation']['GWTolerance'] and steps<v['Simulation']['MaxSteps']:
            choppah = Vehicle(v, m, GW)
            choppah.flyMission()
            missize = choppah.misSize
            if debug: pvar(locals(), ('steps', 'GW', 'missize'))
            GW -= missize
            steps += 1
        if debug: pvar(locals(), ('steps', 'GW', 'missize'))
        if writeOutput: choppah.write()
    
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
            if choppah.misSize > 0:
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
        if GWmax-GWmin <= v['Simulation']['GWTolerance']:
            stopReason = 'Converged to within specified tolerances'
            goodRun = True
        elif steps >= v['Simulation']['MaxSteps']:
            stopReason = 'MaxSteps reached before convergance.  Stopped with bounds: %f  to  %f' % (GWmin, GWmax)
        else:
            stopReason = 'You should never see this text.'
        #choppah.generatePowerCurve()
        #choppah.findHoverCeiling()
        v['Simulation']['StopReason'] = stopReason
        v['Simulation']['GoodRun'] = goodRun
        v['Sizing Results']['SizedGrossWeight'] = GW
        v['Sizing Results']['Optimized'] = True
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