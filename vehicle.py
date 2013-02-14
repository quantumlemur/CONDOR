import math


debug = False
debugFine = False


def pvar(locals_, vars_):
    s = ['%s: %.3f' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)

class Vehicle:
    
    def __init__(self, vconfig, mconfig, GW):
        self.vconfig = vconfig
        self.mconfig = mconfig
        self.GW = GW
        self.setup()

    def setup(self):
        v = self.vconfig
        m = self.mconfig
        GW = self.GW
        self.misSize = None
        diskLoading = v['Main Rotor']['DiskLoading']
        numRotors = v['Main Rotor']['NumRotors']
        v['Body']['FlatPlateDrag'] = 0.25 * GW**.5 * (1-v['Body']['DragTechImprovementFactor']) #0.015 * GW**0.67 # flat plate drag area
        v['Main Rotor']['DiskArea'] = GW / numRotors / diskLoading # disk area per rotor
        v['Main Rotor']['BladeArea'] =  v['Main Rotor']['DiskArea'] *  v['Main Rotor']['Solidity']
        v['Main Rotor']['R'] = math.sqrt(v['Main Rotor']['DiskArea'] / math.pi) # main rotor radius
        v['Wing']['WingSpan'] = v['Main Rotor']['R'] * v['Wing']['SpanRadiusRatio']
        v['Wing']['WingArea'] = v['Wing']['WingSpan']**2 / v['Wing']['WingAspectRatio']
        v['Sizing Results']['MaxBladeLoadingSeen'] = 0
        v['Sizing Results']['BladeLoadingViolated'] = False
        v['Sizing Results']['Nothing'] = 0.
        v['Sizing Results']['HoverCeiling'] = 0.
        
        v['Weights']['EmptyWeight'] = GW * v['Weights']['BaselineEmptyWeightFraction']
        v['Sizing Results']['GrossWeight'] = GW
        
        if v['Simulation']['UseBEMT']:
            self.powerReq = self.BEMT
        else:
            self.powerReq = self.momentumTheory
        self.scaleEngine()
        self.scaleWeights()
        self.setMission(m)


    def scaleWeights(self):
        v = self.vconfig
        w = v['Weights']  # shorthand
        
        # calculation of baseline weight breakdown
        baselineEmptyWeight = w['BaselineEmptyWeight']
        baselineGrossWeight = w['BaselineGrossWeight']
        baselineEngineWeight = w['NumEngines']*w['BaselineWeightPerEngine']
        baselineDriveSystemWeight = w['BaselineWeightPerEngine']*w['BaselineDriveSystemWeightScalingFactor']
        baselineStructureWeight = baselineEmptyWeight - baselineEngineWeight - baselineDriveSystemWeight
        
        # calculation of baseline empty weight fractions
        baselineEmptyWeightFraction = baselineEmptyWeight / baselineGrossWeight
        baselineEngineWeightFraction = baselineEngineWeight / baselineGrossWeight
        baselineDriveSystemWeightFraction = baselineDriveSystemWeight / baselineGrossWeight
        baselineStructureWeightFraction = baselineStructureWeight / baselineGrossWeight
        improvedStructureWeightFraction = baselineStructureWeightFraction * (1-w['StructureWeightTechImprovementFactor'])
        
        # weight scaling
        MCP = v['Powerplant']['MRP'] / 1.3
        scaledEngineWeight = w['NumEngines']*((0.1054*(MCP/w['NumEngines'])**2.+358*(MCP/w['NumEngines'])+2.757*10.**4.)/((MCP/w['NumEngines'])+1180))
        scaledDriveSystemWeight = (525.*(self.GW/1000.)**1.14)/((self.GW/MCP)**0.763*v['Main Rotor']['DiskLoading']**0.381)
        scaledWingWeight = 0.00272*self.GW**1.4/v['Main Rotor']['DiskLoading']**0.8*v['Wing']['SpanRadiusRatio']**0.8
        scaledStructureWeight = improvedStructureWeightFraction * self.GW
        
        v['Weights']['scaledEngineWeight'] = scaledEngineWeight * (1-v['Weights']['EngineWeightTechImprovementFactor'])
        v['Weights']['scaledDriveSystemWeight'] = scaledDriveSystemWeight * (1-v['Weights']['DriveSystemWeightTechImprovementFactor'])
        v['Weights']['scaledWingWeight'] = scaledWingWeight * (1-v['Weights']['WingWeightTechImprovementFactor'])
        v['Weights']['scaledStructureWeight'] = scaledStructureWeight * (1-v['Weights']['StructureWeightTechImprovementFactor'])
        
        # output
        v['Weights']['EmptyWeightFraction'] = (scaledEngineWeight + scaledDriveSystemWeight + scaledWingWeight + scaledStructureWeight) / self.GW
        v['Weights']['EmptyWeight'] = v['Weights']['EmptyWeightFraction'] * self.GW
    
    def setMission(self, mconfig):
        """This is a duplicate of the same function in rf.py"""
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
            m[seg]['Kmu'] = 1. + 3.*m[seg]['AdvanceRatio']**2. + 5.*m[seg]['AdvanceRatio']**4.
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
    
    def generatePowerCurve(self):
        m = self.mconfig
        v = self.vconfig
        seg = 'Segment 1'
        MR = v['Main Rotor'] # shorthand
        speeds = range(0, 200, 1)
        powersSL = [0] * len(speeds)
        powersCruise = [0] * len(speeds)
        # do the actual sweep
        BLs = [0] * len(speeds)
        for i in range(len(speeds)):
            v['Condition']['Weight'] = self.GW
            v['Condition']['Density'] = 0.002378 # SL
            v['Condition']['Speed'] = speeds[i]
            AdvanceRatio =  v['Condition']['Speed']*1.68781 / v['Main Rotor']['TipSpeed']
            v['Condition']['Kmu'] = 1. + 3.*AdvanceRatio**2. + 5.*AdvanceRatio**4.
            v['Condition']['Kl'] = 1 - v['Condition']['Speed']/(1.3*v['Wing']['MaxSpeed'])
            powersSL[i] = self.powerReq()*1.1
        for i in range(len(speeds)):
            v['Condition']['Weight'] = self.GW
            v['Condition']['Density'] = 0.00154522 # 14k ft
            v['Condition']['Speed'] = speeds[i]
            AdvanceRatio =  v['Condition']['Speed']*1.68781 / v['Main Rotor']['TipSpeed']
            v['Condition']['Kmu'] = 1. + 3.*AdvanceRatio**2. + 5.*AdvanceRatio**4.
            v['Condition']['Kl'] = 1 - v['Condition']['Speed']/(1.3*v['Wing']['MaxSpeed'])
            powersCruise[i] = self.powerReq()*1.1
            
            if speeds[i] < v['Wing']['MaxSpeed']:
                drag = .5 * v['Condition']['Density'] * (v['Condition']['Speed']*1.68781)**2 * v['Body']['FlatPlateDrag']
                rotorThrust = math.sqrt(drag**2 + (1*self.GW)**2)
                bladeLoading = rotorThrust / (v['Condition']['Density'] * v['Main Rotor']['BladeArea'] * v['Main Rotor']['TipSpeed']**2)
                BLs[i] = bladeLoading
                v['Sizing Results']['MaxBladeLoadingSeen'] = max(bladeLoading, v['Sizing Results']['MaxBladeLoadingSeen'])
                maxBladeLoading = (200.-v['Condition']['Speed'] / 200.) * .5 + 1.5
                bladeLoadingViolated = bladeLoading > maxBladeLoading
                v['Sizing Results']['BladeLoadingViolated'] = v['Sizing Results']['BladeLoadingViolated'] or bladeLoadingViolated
            
        v['BLs'] = BLs
        self.speeds = speeds
        self.powersCruise = powersCruise
        self.powersSL = powersSL
    
    def findHoverCeiling(self):
        v = self.vconfig
        m = self.mconfig
        powerAvailable = 999999
        powerRequired = 0
        altitude = 0
        while powerAvailable > powerRequired:
            altitude += 100
            v['Condition']['Weight'] = self.GW
            v['Condition']['Density'] = 5e-13*altitude**2 - 7e-8*altitude + .0024
            v['Condition']['Speed'] = 0.
            AdvanceRatio =  v['Condition']['Speed']*1.68781 / v['Main Rotor']['TipSpeed']
            v['Condition']['Kmu'] = 1. + 3.*AdvanceRatio**2. + 5.*AdvanceRatio**4.
            v['Condition']['Kl'] = 1 - v['Condition']['Speed']/(1.3*v['Wing']['MaxSpeed'])
            powerRequired = self.powerReq()*1.1
            powerAvailable = v['Powerplant']['MCP'] * (-2e-5*altitude + 1)
        v['Sizing Results']['HoverCeiling'] = altitude
    
    def findMaxRange(self):
        """Finds and stores the speed for max range and that range.  Must be run after generatePowerCurve()"""
        speeds = self.speeds
        powers = self.powers
        SPEEDmaxr = 0
        POWERmaxr = 999999.
        imin = 9999999.
        for i in range(len(speeds)):
            if powers[i]/speeds[i] < imin:
                imin = powers[i]/speeds[i]
                SPEEDmaxr = speeds[i]
                POWERmaxr = powers[i]
        if debug:  print speeds
        if debug:  print powers
        fuelweight = self.GW - (self.vconfig['Weights']['EmptyWeightFraction']*self.GW + self.vconfig['Weights']['UsefulLoad'])
        hourstoempty = fuelweight / (self.SFC(POWERmaxr) * POWERmaxr)
        self.maxrange = hourstoempty * SPEEDmaxr
        self.SPEEDmaxr = SPEEDmaxr
        self.POWERmaxr = POWERmaxr
    
    def scaleEngine(self):
        """Scales the engine for high hot hover and fast cruise."""
        v = self.vconfig
        v['Condition']['Weight'] = self.GW
        v['Condition']['Density'] = 0.001852 # 6k 95f
        v['Condition']['Speed'] = 0 # hover
        AdvanceRatio =  v['Condition']['Speed']*1.68781 / v['Main Rotor']['TipSpeed']
        v['Condition']['Kmu'] = 1 + 3*AdvanceRatio**2 + 5*AdvanceRatio**4
        v['Condition']['Kl'] = 1 - v['Condition']['Speed']/(1.3*v['Wing']['MaxSpeed'])
        hoverpower = self.powerReq()
        altitude = 6000.
        temp = 62.6
        hoverpower = hoverpower / ((1-0.195*(altitude/10000))*(1-0.005*(temp-59))) # scale engine to sea level
        
        v['Condition']['Weight'] = self.GW
        v['Condition']['Density'] = 0.001207 # 20k 95f
        v['Condition']['Speed'] = v['Wing']['MaxSpeed'] # Max speed
        AdvanceRatio =  v['Condition']['Speed']*1.68781 / v['Main Rotor']['TipSpeed']
        v['Condition']['Kmu'] = 1 + 3*AdvanceRatio**2 + 5*AdvanceRatio**4
        v['Condition']['Kl'] = 1 - v['Condition']['Speed']/(1.3*v['Wing']['MaxSpeed'])
        cruisepower = self.powerReq()
        altitude = 20000.
        temp = 15.8
        cruisepower = cruisepower / ((1-0.195*(altitude/10000))*(1-0.005*(temp-59))) # scale engine to sea level
        
        power = max(hoverpower, cruisepower)
        gamma = power / self.vconfig['Powerplant']['BaselineMRP']
        sfc = (-0.00932*gamma**2+0.865*gamma+0.445)/(gamma+0.301)*v['Powerplant']['BaselineSFC']
        
        v['Powerplant']['MRP'] = power * 1.3
        v['Powerplant']['MCP'] = power 
        v['Powerplant']['SFC'] = sfc * (1-v['Powerplant']['SFCTechImprovementFactor'])
    
    def SFC(self, power):
        """Returns SFC at a given output power."""
        v = self.vconfig
        # TODO:  Insert partial-power SFC
        return v['Powerplant']['SFC']
    
    def flyMission(self):
        m = self.mconfig
        v = self.vconfig
        # we will simulate running the mission backwards, starting from an empty fuel tank
        elapsed = m['TotalTime'] # elapsed time since mission start
        totalFuel = 0 # total fuel used, pounds
        w = v['Weights']['EmptyWeight']
        maxGW = w
        MR = v['Main Rotor'] # shorthand
        # step through all the segments specified in the mission file
        for i in range(m['NumSegments'], 0, -1):
            seg = 'Segment %d' % i
            w += m[seg]['DeltaLoad'] # add or subtract and weight changes specified in the mission
            if debug: print 'Starting at end of %s    adding load: %d     weight: %d' % (seg, m[seg]['DeltaLoad'], w)
            # Since we're simulating backwards, we don't want to underestimate our sizing,
            # by performing each step with the weight at the end of the step, rather than
            # at the beginning.  So, we'll run the first two steps in each segment,
            # providing us with a better estimate that we can use to re-run the first step,
            # and making sure we don't underestimate the size, especially with a large
            # simulation step.  Basically we're running each step with an estimate of what
            # the weight will be at the beginning of it.
            v['Condition']['Weight'] = w
            v['Condition']['Segment'] = seg
            def makeCurrent(section, key, vconfig):
                vconfig['Condition'][key] = section[key]
            m[seg].walk(makeCurrent, vconfig=v)
            power = self.powerReq()
            duration = v['Simulation']['TimeStep']
            # Check if we're about to overfly the segment
            if elapsed-v['Simulation']['TimeStep'] < m[seg]['StartTime']:
                duration = elapsed - m[seg]['StartTime']
            fuel = self.SFC(power) * power * (duration/60.)
            nextW = w + fuel
            # Now that we have our estimate, we can start looping through the segment for real.
            while elapsed > m[seg]['StartTime']: # keep stepping through until we finish a segment
                if debugFine: print 'StartTime: %f     Elapsed: %f     EndTime: %f' % (m[seg]['EndTime'], elapsed, m[seg]['EndTime'])
                duration = v['Simulation']['TimeStep']
                # Check if we're about to overfly the segment
                if elapsed-v['Simulation']['TimeStep'] < m[seg]['StartTime']:
                    duration = elapsed - m[seg]['StartTime']
                    if debugFine: print 'First segment bit, duration %s minutes' % duration
                v['Condition']['Weight'] = nextW
                v['Condition']['Segment'] = seg
                m[seg].walk(makeCurrent, vconfig=v)
                power = self.powerReq()
                fuel = self.SFC(power) * power * (duration/60)
                w += fuel
                maxGW = max(w, maxGW)
                nextW = w + fuel
                totalFuel += fuel
                elapsed -= duration
            if debug: print('%s RfR: %.3f' % (seg, totalFuel/self.GW))
        maxGW = max(maxGW, v['Weights']['EmptyWeight']+v['Weights']['UsefulLoad']+totalFuel)
        if debug: print 'Finished!  Total fuel used: %s     Missize amount: %s     RfA: %.3f' % (totalFuel, self.GW-maxGW, 1-(v['Weights']['UsefulLoad']/self.GW)-v['Weights']['EmptyWeightFraction'])
        self.misSize = self.GW-maxGW
        v['Sizing Results']['MisSize'] = self.misSize

    def momentumTheory(self):
        v = self.vconfig
        m = self.mconfig
        MR = v['Main Rotor']
        w = v['Condition']['Weight']
        R = MR['R']
        TipLossFactor=MR['TipLossFactor']
        Density=v['Condition']['Density']
        Kint=MR['Kint']
        Kov=MR['Kov']
        BladeArea=MR['BladeArea']
        FlatPlateDrag=v['Body']['FlatPlateDrag']
        Speed=v['Condition']['Speed']
        PropEfficiency=v['Aux Propulsion']['PropEfficiency']
        OswaldEfficiency=v['Wing']['OswaldEfficiency']
        WingSpan=v['Wing']['WingSpan']
        NumRotors=MR['NumRotors']
        Kmu=v['Condition']['Kmu']
        HeliEfficiency=MR['HeliEfficiency']
        CD0=MR['CD0']
        TipSpeed=MR['TipSpeed']
        Kl=v['Condition']['Kl']
        
        # wing lift equations adapted from CIRADS
        WingLift = 0.5 * v['Wing']['WingArea'] * Density * (Speed*1.68781)**2 * v['Wing']['WingClMax'] # wing produces as much lift as it can
        WingLift = min(.9*w, WingLift) # ...up to 90% of the weight
        #if v['Wing']['WingArea']>0 and Speed>0:
        #    WingCl = WingLift / (.5 * v['Wing']['WingArea'] * Density * (Speed*1.68781)**2) # actual wing Cl
        #    WingCd = v['Wing']['WingCD0'] + WingCl**2/(v['Wing']['OswaldEfficiency'] * math.pi * v['Wing']['WingAspectRatio']) + .0001*(WingCl/v['Wing']['WingClMax'])**100   # wing cd w/ stall
        #else:
        #    WingCl = 0
        #    WingCd = 0
        #WingDrag = .5 * WingCd * v['Wing']['WingArea'] * Density * (Speed*1.68781)**2
        #WingPower = WingDrag * (Speed*1.68781) / 550 / PropEfficiency
        Kl = (w - WingLift) / w
        
        if Kl<0: Kl = 0
        if Kl>1: Kl = 1
        
        DensityRatio = Density / 0.002377
        #DiskLoading = w / (NumRotors * math.pi * R**2)
        DiskLoading = MR['DiskLoading']
        inducedPower = NumRotors * Kint * Kov * (0.0938 * DiskLoading**1.5 * R**2) / (TipLossFactor*DensityRatio**0.5) # in horsepower
        profilePower = NumRotors * DensityRatio * BladeArea * (TipSpeed/100)**3 * CD0 / 1.85
        THP1 = (DensityRatio * FlatPlateDrag * (Speed*1)**3.1) / (146000*PropEfficiency) #1.15078
        THP2 = ((0.332 / (DensityRatio * OswaldEfficiency * PropEfficiency * (Speed*1.15078))) * ((1-Kl) * w / WingSpan)**2) if (Speed>0 and WingSpan>0) else 0
        inducedVelocity = math.sqrt(DiskLoading / (2*Density))
        Ku = math.sqrt((math.sqrt(((Speed*1.68781) / inducedVelocity)**4 + 4) - ((Speed*1.68781) / inducedVelocity)**2) / 2)
        THP3 = (Kl**1.5 * Ku * inducedPower + Kmu * profilePower) / HeliEfficiency
        # TODO:  Add tail rotor power
        totalPower = THP1 + THP2 + THP3
        
        if debugFine: pvar(locals(), ('Kint', 'Kov', 'DiskLoading', 'R', 'TipLossFactor', 'Density'))
        if debugFine: pvar(locals(), ('Kl', 'Ku', 'inducedPower', 'Kmu', 'profilePower', 'HeliEfficiency'))
        if debug: pvar(locals(), ('w', 'DiskLoading', 'inducedPower', 'profilePower', 'THP1', 'THP2', 'THP3', 'totalPower'))
        return totalPower
    
    def BEMT(self, w, R, TipLossFactor, Density, Kint, Kov, BladeArea, FlatPlateDrag, Speed, PropEfficiency, OswaldEfficiency, WingSpan, NumRotors, Kmu, HeliEfficiency, CD0, TipSpeed, Kl):
        pass

    def write(self):
        # write out the output
        v = self.vconfig
        v.filename = 'Config/output.cfg'
        v.write()
        
        m = self.mconfig
        m.filename = 'Config/missionout.cfg'
        m.write()


if __name__ == '__main__':   
    from configobj import ConfigObj
    from validate import Validator
    v = ConfigObj('Config/vehicle.cfg', configspec='Config/vehicle.configspec')
    m = ConfigObj('Config/mission.cfg', configspec='Config/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    vehicle = Vehicle(v, m, 30000.)
    vehicle.flyMission()
    vehicle.write()
    