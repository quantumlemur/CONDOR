import math
from BEMT import Blade, Rotor

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
        v['Body']['FlatPlateDrag'] = 0.27 * GW**.5 * (1-v['Body']['DragTechImprovementFactor']) #0.015 * GW**0.67 # flat plate drag area
        v['Main Rotor']['Omega'] = v['Main Rotor']['TipSpeed'] / v['Main Rotor']['Radius']
        v['Main Rotor']['DiskArea'] = math.pi * v['Main Rotor']['Radius']**2 * v['Main Rotor']['NumRotors']
        v['Main Rotor']['DiskLoading'] = GW / v['Main Rotor']['NumRotors'] / v['Main Rotor']['DiskArea']
        v['Wing']['WingSpan'] = v['Main Rotor']['Radius'] * v['Wing']['SpanRadiusRatio']
        v['Wing']['WingArea'] = v['Wing']['WingSpan']**2 / v['Wing']['WingAspectRatio']
        
        v['Weights']['EmptyWeight'] = GW * v['Weights']['BaselineEmptyWeightFraction']
        v['Sizing Results']['GrossWeight'] = GW
        v['Sizing Results']['CouldTrim'] = True
        
        self.blade = Blade(c81File='Config/%s'%v['Main Rotor']['AirfoilFile'], skip_header=0, skip_footer=0, rootChord=v['Main Rotor']['RootChord']/v['Main Rotor']['Radius'], taperRatio=v['Main Rotor']['TaperRatio'], tipTwist=v['Main Rotor']['TipTwist'], rootCutout=v['Main Rotor']['RootCutout']/v['Main Rotor']['Radius'], segments=v['Simulation']['numBladeElementSegments'], dragDivergenceMachNumber=v['Main Rotor']['DragDivergenceMachNumber'])
        self.rotor = Rotor(self.blade, psiSegments=v['Simulation']['numBladeElementSegments'], Vtip=v['Main Rotor']['TipSpeed'], radius=v['Main Rotor']['Radius'], numblades=v['Main Rotor']['NumBlades'])
        v['Main Rotor']['Solidity'] = self.rotor.solidity
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
        """This function sets the mission and analyzes all the mission segments, fleshing them out with some calculated values"""
        self.mconfig = mconfig
        m = self.mconfig
        v = self.vconfig
        totaltime = 0
        segment = 1
        numsegs = 0
        prevLoad = 0
        while 'Segment %s' % segment in m:
            seg = 'Segment %s' % segment
            m[seg]['Time'] = m[seg]['Distance'] / m[seg]['Speed'] * 60 # time in minutes
            m[seg]['StartTime'] = totaltime
            totaltime += m[seg]['Time']
            m[seg]['EndTime'] = totaltime
            m[seg]['Density'] = self.density(m[seg]['Altitude'])
            m[seg]['DensityRatio'] = m[seg]['Density'] / self.density(0)
            load = m[seg]['CrewWeight'] + m[seg]['PayloadWeight'] + m[seg]['MiscWeight']
            m[seg]['DeltaLoad'] = load - prevLoad
            prevLoad = load
            segment += 1
            numsegs += 1
        m['TotalTime'] = totaltime
        m['NumSegments'] = numsegs

    def generatePowerCurve(self):
        m = self.mconfig
        v = self.vconfig
        seg = 'Segment 1'
        MR = v['Main Rotor'] # shorthand
        v['Condition']['Weight'] = self.GW
        speeds = [0]
        powersSL = []
        powersCruise = []
        # Find hover power and start out the arrays
        v['Condition']['Density'] = self.density(0) # SL
        v['Condition']['Speed'] = speeds[0]
        powersSL.append(self.powerReq())
        # Do the sea level sweep
        while not math.isnan(powersSL[-1]):
            speed = speeds[-1] + v['Simulation']['PowerCurveResolution']
            v['Condition']['Speed'] = speed
            v['Condition']['Density'] = self.density(0) # SL
            powersSL.append(float('nan')) #self.powerReq())
            powersCruise.append(float('nan'))
            speeds.append(speed)
        # Find cruise hover power and start out the array
        v['Condition']['Density'] = self.density(v['Condition']['CruiseAltitude']) # 10k ft
        v['Condition']['Speed'] = speeds[0]
        powersCruise.append(self.powerReq())
        # Do the cruise sweep
        i = 1
        while (not math.isnan(powersCruise[-1])) or i<len(speeds):
            speed = speeds[i-1] + v['Simulation']['PowerCurveResolution']
            v['Condition']['Speed'] = speed
            v['Condition']['Density'] = self.density(v['Condition']['CruiseAltitude']) # 10k ft
            power = self.powerReq()
            if speed in speeds:
                powersCruise[i] = power
            else:
                speeds.append(speed)
                powersSL.append(float('nan'))
                powersCruise.append(power)
            i += 1
        # Store the power curves
        v['Power Curve']['Speeds'] = speeds
        v['Power Curve']['PowersCruise'] = powersCruise
        v['Power Curve']['PowersSL'] = powersSL
    
    def findHoverCeiling(self):
        v = self.vconfig
        m = self.mconfig
        powerAvailable = 999999
        powerRequired = 0
        altitude = 0
        v['Condition']['Weight'] = self.GW
        v['Condition']['Speed'] = 0.
        while (powerAvailable>powerRequired) and (not math.isnan(powerRequired)):
            altitude += v['Simulation']['HoverCeilingResolution']
            v['Condition']['Density'] = self.density(altitude)
            powerRequired = self.powerReq()
            powerAvailable = self.powerAvailable(altitude)
        v['Performance']['HoverCeiling'] = altitude
    
    def density(self, altitude):
        return 5e-13*altitude**2 - 7e-8*altitude + .0024 # ISA+15C?

    def powerAvailable(self, altitude):
        return self.vconfig['Powerplant']['MCP'] * self.density(altitude) / self.density(0)#(-2e-5*altitude + 1)

    def speedOfSound(self, density):
        return 1026. # yeah... replace this with an equation

    def findMaxRange(self):
        """Finds and stores the speed for max range and that range.  Must be run after generatePowerCurve()"""
        v = self.vconfig
        speeds = v['Power Curve']['Speeds']
        powers = v['Power Curve']['PowersCruise']
        SPEEDmaxr = 0.
        POWERmaxr = 999999.
        imin = 9999999.
        for i in range(1, len(speeds)):
            if powers[i]/speeds[i] < imin:
                imin = powers[i]/speeds[i]
                SPEEDmaxr = speeds[i]
                POWERmaxr = powers[i]
        if debug:  print speeds
        if debug:  print powers
        fuelweight = self.GW - (v['Weights']['EmptyWeightFraction']*self.GW + v['Weights']['UsefulLoad'])
        hourstoempty = fuelweight / (self.SFC(POWERmaxr) * POWERmaxr)
        v['Weights']['MaxAvailableFuelWeight'] = fuelweight
        v['Performance']['MaxRange'] = hourstoempty * SPEEDmaxr
        v['Performance']['MaxRangeSpeed'] = SPEEDmaxr
        v['Performance']['PowerAtMaxRangeSpeed'] = POWERmaxr

    def findMaxSpeed(self):
        """Finds and stores the maximum speed.  Must be run after generatePowerCurve()"""
        v = self.vconfig
        powerAvailable = self.powerAvailable(v['Condition']['CruiseAltitude'])
        speeds = v['Power Curve']['Speeds']
        powers = v['Power Curve']['PowersCruise']
        maxSpeed = 0.
        maxSpeedPower = 0.
        for i in range(len(speeds)):
            if powers[i]<powerAvailable and speeds[i]>maxSpeed:
                maxSpeed = speeds[i]
                maxSpeedPower = powers[i]
        v['Performance']['MaxSpeed'] = maxSpeed
        v['Performance']['PowerAtMaxSpeed'] = maxSpeedPower

    
    def scaleEngine(self):
        """Scales the engine for high hot hover and fast cruise."""
        v = self.vconfig
        v['Condition']['Weight'] = self.GW
        v['Condition']['Density'] = self.density(6000) # 0.001852 # 6k 95f
        v['Condition']['Speed'] = 0 # hover
        hoverpower = self.powerReq()
        if math.isnan(hoverpower):
            v['Sizing Results']['CouldTrim'] = False
        altitude = 6000.
        temp = 62.6
        hoverpower = hoverpower / ((1-0.195*(altitude/10000))*(1-0.005*(temp-59))) # scale engine to sea level
        
        v['Condition']['Weight'] = self.GW
        v['Condition']['Density'] = self.density(13000) #0.001207 # 20k 95f
        v['Condition']['Speed'] = v['Wing']['MaxSpeed'] # Max speed
        cruisepower = self.powerReq()
        if math.isnan(cruisepower):
            v['Sizing Results']['CouldTrim'] = False
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
        #return v['Powerplant']['SFC']
        return -0.00001495*power + .4904 # right now this is the GE CT7-8 engine, in the S-92.  Put it back to a generic scaling using v['Powerplant']['SFC'] !
    
    def flyMission(self):
        m = self.mconfig
        v = self.vconfig
        elapsed = 0 # elapsed time since mission start
        fuelAvailable = self.GW - v['Weights']['EmptyWeight'] - v['Weights']['UsefulLoad'] # total fuel weight available, pounds
        w = v['Weights']['EmptyWeight'] + fuelAvailable
        totalFuel = 0.
        MR = v['Main Rotor'] # shorthand
        # step through all the segments specified in the mission file
        for i in range(0, m['NumSegments']):
            if not v['Sizing Results']['CouldTrim']:
                break
            seg = 'Segment %d' % (i+1)
            w += m[seg]['DeltaLoad'] # add or subtract and weight changes specified in the mission
            if debug: print 'Starting at %s    adding load: %d     weight: %d' % (seg, m[seg]['DeltaLoad'], w)
            while elapsed < m[seg]['EndTime']: # keep stepping through until we finish a segment
                if debugFine: print 'StartTime: %f     Elapsed: %f     EndTime: %f' % (m[seg]['StartTime'], elapsed, m[seg]['EndTime'])
                duration = v['Simulation']['TimeStep']
                # Check if we're about to overfly the segment
                if elapsed+v['Simulation']['TimeStep'] > m[seg]['EndTime']:
                    duration = m[seg]['EndTime'] - elapsed
                    if debugFine: print 'Last segment bit, duration %s minutes' % duration
                v['Condition']['Weight'] = w
                power = self.powerReq()
                if math.isnan(power):
                    v['Sizing Results']['CouldTrim'] = False
                    break
                fuel = self.SFC(power) * power * (duration/60)
                w -= fuel
                totalFuel += fuel
                elapsed += duration
            if debug: print('%s RfR: %.3f' % (seg, totalFuel/self.GW))
        self.misSize = fuelAvailable - totalFuel
        if debug: print 'Finished!  Total fuel used: %s     Missize amount: %s' % (totalFuel, self.misSize)
        v['Sizing Results']['MisSize'] = self.misSize

    
    def powerReq(self):
        v = self.vconfig
        Density = v['Condition']['Density']
        V = v['Condition']['Speed'] * 1.687 # speed in feet per second

        # proportion out the vertical lift between the rotors and the wing
        VerticalLift_wing = 0.5 * v['Wing']['WingArea'] * Density * V**2 * v['Wing']['WingClMax'] # wing produces as much lift as it can
        VerticalLift_wing = min(v['Wing']['MaxWingLoadProportion']*v['Condition']['Weight'], VerticalLift_wing) # ...up to 90% of the weight
        VerticalLift_rotors = v['Condition']['Weight'] - VerticalLift_wing
        VerticalLift_perRotor = VerticalLift_rotors / v['Main Rotor']['NumRotors']

        # calculate wing drag
        if v['Wing']['WingArea']>0 and V>0:
            WingCl = VerticalLift_wing / (.5 * v['Wing']['WingArea'] * Density * V**2) # actual wing Cl
            WingCd = v['Wing']['WingCD0'] + WingCl**2/(v['Wing']['OswaldEfficiency'] * math.pi * v['Wing']['WingAspectRatio'])# + .0001*(WingCl/v['Wing']['WingClMax'])**100   # wing cd w/ stall
        else:
            WingCl = 0.
            WingCd = 0.
        WingDrag = .5 * WingCd * v['Wing']['WingArea'] * Density * V**2

        # proportion out forward thrust between the aux prop and the rotors
        BodyDrag = .5 * Density * V**2 * v['Body']['FlatPlateDrag']
        ForwardThrust = BodyDrag + WingDrag
        if v['Aux Propulsion']['NumAuxProps'] > 0:
            ForwardThrust_auxprops = ForwardThrust * v['Aux Propulsion']['PropulsionFactor']
            ForwardThrust_perAuxprop = ForwardThrust_auxprops / v['Aux Propulsion']['NumAuxProps']
        else:
            ForwardThrust_auxprops = 0.
            ForwardThrust_perAuxprop = 0.
        ForwardThrust_rotors = ForwardThrust - ForwardThrust_auxprops
        ForwardThrust_perRotor = ForwardThrust_rotors / v['Main Rotor']['NumRotors']

        # calculate rotor power
        singleRotorPower = self.rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], V=V, rho=Density, speedOfSound=self.speedOfSound(Density), Fx=ForwardThrust_perRotor, Fz=VerticalLift_perRotor, maxSteps=v['Simulation']['MaxSteps'])
        if math.isnan(singleRotorPower):
            v['Trim Failure']['V'] = V / 1.687
            v['Trim Failure']['rho'] = Density
            v['Trim Failure']['speedOfSound'] = self.speedOfSound(Density)
            v['Trim Failure']['Fx'] = ForwardThrust_perRotor
            v['Trim Failure']['Fz'] = VerticalLift_perRotor
            v['Trim Failure']['MaxSteps'] = v['Simulation']['MaxSteps']

        # calculate prop power
        singlePropPower = ForwardThrust_perAuxprop * V * v['Aux Propulsion']['PropEfficiency'] / 550

        # find total power
        totalPower = singleRotorPower*v['Main Rotor']['NumRotors'] + singlePropPower*v['Aux Propulsion']['NumAuxProps']
        totalPower = totalPower / (1-v['Antitorque']['AntitorquePowerFactor']) / (1-v['Powerplant']['TransmissionEfficiency'])
        if debug: pvar(locals(), ('singleRotorPower', 'totalPower'))
        return totalPower
        

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
    import matplotlib.pyplot as plt
    v = ConfigObj('Config/vehicle.cfg', configspec='Config/vehicle.configspec')
    m = ConfigObj('Config/mission.cfg', configspec='Config/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    vehicle = Vehicle(v, m, 26000.)
    #vehicle.flyMission()
    vehicle.generatePowerCurve()
    vehicle.findHoverCeiling()
    vehicle.findMaxRange()
    vehicle.findMaxSpeed()
    vehicle.write()

    plt.figure()
    plt.plot(v['Power Curve']['Speeds'], v['Power Curve']['PowersSL'])
    plt.plot(v['Power Curve']['Speeds'], v['Power Curve']['PowersCruise'])
    MCPspeeds = [0, 200]
    MCPSL = [vehicle.powerAvailable(0), vehicle.powerAvailable(0)]
    MCPalt = [vehicle.powerAvailable(v['Condition']['CruiseAltitude']), vehicle.powerAvailable(v['Condition']['CruiseAltitude'])]
    plt.plot(MCPspeeds, MCPSL, 'b')
    plt.plot(MCPspeeds, MCPalt, 'g')
    plt.legend(('Sea Level', 'Cruise'))
    plt.axis([0, 200, 0, 5000])
    plt.xlabel('Speed (kts)')
    plt.ylabel('Power (hp)')
    plt.show()

    