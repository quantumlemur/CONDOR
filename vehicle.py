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
from configobj import ConfigObj
from validate import Validator
from BEMT import Blade, Rotor

# BEGIN SCRIPT CONFIG BLOCK
debug = False
debugFine = False
vehicleConfigPath = 'Config/vehicle_s92.cfg'
missionConfigPath = 'Config/AHS_mission3.cfg'
# END SCRIPT CONFIG BLOCK


def pvar(locals_, vars_):
    s = ['%s: %.3f' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)

class Vehicle:

    def __init__(self, vconfig, mconfig, GW, airfoildata_mainRotor, airfoildata_auxProp):
        self.vconfig = ConfigObj(vconfig)
        self.mconfig = ConfigObj(mconfig)
        self.GW = GW
        self.airfoildata_mainRotor = airfoildata_mainRotor
        self.airfoildata_auxProp = airfoildata_auxProp
        self.setup()

    def setup(self):
        v = self.vconfig
        m = self.mconfig
        GW = self.GW
        self.misSize = float('nan')

        # Engine scaling
        v['Condition']['CruiseAltitude'] = m['Segment 2']['Altitude']
        v['Engine Scaling']['RequiredHoverHeight'] = m['Segment 1']['Altitude']
        v['Wing']['MaxSpeed'] = m['Segment 2']['Speed']

        v['Performance']['IngressSpeed'] = m['Segment 2']['Speed'] # REMOVEME
        v['Performance']['MissionRange'] = m['Segment 2']['Distance'] # REMOVEME

        v['Performance']['MaxBladeLoadingSeen'] = 0.

        v['Body']['FlatPlateDrag'] = 0.25 * GW**.5 * (1-v['Body']['DragTechImprovementFactor']) #0.015 * GW**0.67 # flat plate drag area
        v['Main Rotor']['Radius'] = math.sqrt(GW / (math.pi * v['Main Rotor']['DiskLoading'] * v['Main Rotor']['NumRotors']))
        v['Main Rotor']['Omega'] = v['Main Rotor']['TipSpeed'] / v['Main Rotor']['Radius']
        v['Main Rotor']['DiskArea'] = math.pi * v['Main Rotor']['Radius']**2 * v['Main Rotor']['NumRotors']
        v['Main Rotor']['AverageChord'] = v['Main Rotor']['DiskArea']*v['Main Rotor']['Solidity'] / (v['Main Rotor']['Radius']*(1-v['Main Rotor']['RootCutout'])*v['Main Rotor']['NumBlades'])

        v['Aux Propulsion']['Omega'] = v['Aux Propulsion']['TipSpeed'] / v['Aux Propulsion']['Radius']
        v['Aux Propulsion']['DiskArea'] = math.pi * v['Aux Propulsion']['Radius']**2 * v['Aux Propulsion']['NumAuxProps']
        v['Aux Propulsion']['AverageChord'] = v['Aux Propulsion']['DiskArea']*v['Aux Propulsion']['Solidity'] / (v['Aux Propulsion']['Radius']*(1-v['Aux Propulsion']['RootCutout'])*v['Aux Propulsion']['NumBlades'])


        v['Wing']['WingSpan'] = v['Main Rotor']['Radius'] * v['Wing']['SpanRadiusRatio']
        v['Wing']['WingArea'] = v['Wing']['WingSpan'] * v['Wing']['WingChord']
        v['Wing']['WingAspectRatio'] = v['Wing']['WingSpan'] / v['Wing']['WingChord']

        v['Sizing Results']['GrossWeight'] = GW
        v['Sizing Results']['CouldTrim'] = True
        v['Sizing Results']['MisSize'] = float('nan')
        v['Sizing Results']['Nothing'] = 0.
        v['Performance']['Nothing'] = 0.

        # adjustments and tweaks based on configuration
        # if v['Main Rotor']['NumRotors'] == 2:
        #     v['Antitorque']['AntitorquePowerFactor'] = 0.
        #     v['Powerplant']['TransmissionEfficiency'] = .08
        #     v['Weights']['BaselineEmptyWeightFraction'] = v['Weights']['BaselineEmptyWeightFraction'] * 1.05
        #     v['Body']['FlatPlateDrag'] = v['Body']['FlatPlateDrag'] * 1.1


        self.blade = Blade(airfoildata=self.airfoildata_mainRotor, skip_header=0, skip_footer=0, averageChord=v['Main Rotor']['AverageChord']/v['Main Rotor']['Radius'], taperRatio=v['Main Rotor']['TaperRatio'], tipTwist=v['Main Rotor']['TipTwist'], rootCutout=v['Main Rotor']['RootCutout'], segments=v['Simulation']['numBladeElementSegments'], dragDivergenceMachNumber=v['Main Rotor']['DragDivergenceMachNumber'])
        self.rotor = Rotor(self.blade, psiSegments=v['Simulation']['numBladeElementSegments'], Vtip=v['Main Rotor']['TipSpeed'], radius=v['Main Rotor']['Radius'], numblades=v['Main Rotor']['NumBlades'])
        self.blade_aux = Blade(airfoildata=self.airfoildata_auxProp, skip_header=0, skip_footer=0, averageChord=v['Aux Propulsion']['AverageChord']/v['Aux Propulsion']['Radius'], taperRatio=v['Aux Propulsion']['TaperRatio'], tipTwist=v['Aux Propulsion']['TipTwist'], rootCutout=v['Aux Propulsion']['RootCutout'], segments=v['Simulation']['numBladeElementSegments'], dragDivergenceMachNumber=v['Aux Propulsion']['DragDivergenceMachNumber'])
        self.rotor_aux = Rotor(self.blade_aux, psiSegments=v['Simulation']['numBladeElementSegments'], Vtip=v['Aux Propulsion']['TipSpeed'], radius=v['Aux Propulsion']['Radius'], numblades=v['Aux Propulsion']['NumBlades'])
        self.scaleEngine()
        self.scaleWeights()
        self.findCost()
        self.setMission(m)

    def findCost(self):
        v = self.vconfig
        HarrisScullyPrice = 628.707 * (v['Main Rotor']['NumBlades']*v['Main Rotor']['NumRotors'])**0.2045 * v['Weights']['EmptyWeight']**0.4854 * v['Powerplant']['MRP']**0.5843
        v['Economics']['HarrisScullyPrice'] = HarrisScullyPrice

    def OEC(self):
        v = self.vconfig
        usefulLoad = v['Weights']['MaxAvailableFuelWeight']
        sfc = v['Performance']['SFCatMaxRange']
        fuelConsumption = v['Performance']['SFCatMaxRange'] * v['Performance']['PowerAtMaxRangeSpeed']
        if v['Performance']['MaxRangeSpeed'] == 0.:
            payload = 0.
        else:
            timeToRange = v['OEC']['BaselineRange'] / v['Performance']['MaxRangeSpeed']
            fuelConsumed = fuelConsumption * timeToRange
            payload = usefulLoad - fuelConsumed
        v['OEC']['Payload'] = payload
        pFactor = payload / v['OEC']['BaselinePayload']
        rFactor = v['Performance']['MaxRange'] / v['OEC']['BaselineRange']
        hFactor = v['Performance']['HoverCeiling'] / v['OEC']['BaselineHoverCeiling']
        sFactor = v['Performance']['MaxSpeed'] / v['OEC']['BaselineSpeed']
        cFactor = v['OEC']['BaselineCost'] / v['Economics']['HarrisScullyPrice']
        v['OEC']['MCI'] = rFactor + hFactor + sFactor + pFactor
        v['OEC']['OEC'] = rFactor + hFactor + sFactor + pFactor + cFactor

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
        MRP = v['Powerplant']['MRP']
        scaledEngineWeight = w['NumEngines']*((0.1054*(MRP/w['NumEngines'])**2.+358*(MRP/w['NumEngines'])+2.757*10.**4.)/((MRP/w['NumEngines'])+1180))
        try:
            scaledDriveSystemWeight = (525.*(self.GW/1000.)**1.14)/((self.GW/MRP)**0.763*v['Main Rotor']['DiskLoading']**0.381)
        except:
            scaledDriveSystemWeight = 999999999.
            print scaledDriveSystemWeight
            print MRP
        scaledWingWeight = 0.00272*self.GW**1.4 / v['Main Rotor']['DiskLoading']**0.8 * v['Wing']['SpanRadiusRatio']**0.8 * 0.8
        scaledStructureWeight = improvedStructureWeightFraction * self.GW

        v['Weights']['scaledEngineWeight'] = scaledEngineWeight * (1-v['Weights']['EngineWeightTechImprovementFactor'])
        v['Weights']['scaledDriveSystemWeight'] = scaledDriveSystemWeight * (1-v['Weights']['DriveSystemWeightTechImprovementFactor'])
        v['Weights']['scaledWingWeight'] = scaledWingWeight * (1-v['Weights']['WingWeightTechImprovementFactor'])
        v['Weights']['scaledStructureWeight'] = scaledStructureWeight * (1-v['Weights']['StructureWeightTechImprovementFactor'])

        # output
        v['Weights']['EmptyWeightFraction'] = (scaledEngineWeight + scaledDriveSystemWeight + scaledWingWeight + scaledStructureWeight) / self.GW - 0.11
        v['Weights']['EmptyWeight'] = v['Weights']['EmptyWeightFraction'] * self.GW
        v['Weights']['MaxAvailableFuelWeight'] = self.GW - v['Weights']['EmptyWeight'] #- v['Weights']['UsefulLoad']

    def setMission(self, mconfig):
        """This function sets the mission and analyzes all the mission segments, fleshing them out with some calculated values"""
        self.mconfig = mconfig
        m = self.mconfig
        v = self.vconfig
        totaltime = 0
        segment = 1
        numsegs = 0
        prevLoad = 0
        maxLoad = 0
        totalRange = 0
        while 'Segment %s' % segment in m:
            seg = 'Segment %s' % segment
            m[seg]['Time'] = m[seg]['Distance'] / m[seg]['Speed'] * 60 # time in minutes
            m[seg]['StartTime'] = totaltime
            totaltime += m[seg]['Time']
            m[seg]['EndTime'] = totaltime
            m[seg]['Density'] = self.density(m[seg]['Altitude'])
            m[seg]['DensityRatio'] = m[seg]['Density'] / self.density(0)
            load = m[seg]['CrewWeight'] + m[seg]['PayloadWeight'] + m[seg]['MiscWeight']
            maxLoad = max(maxLoad, load)
            totalRange += m[seg]['Distance']
            m[seg]['DeltaLoad'] = load - prevLoad
            prevLoad = load
            segment += 1
            numsegs += 1
        v['Sizing Results']['TotalRange'] = totalRange
        v['Sizing Results']['Payload'] = m['Segment 2']['PayloadWeight']
        v['Sizing Results']['TotalWeight'] = float('nan')
        v['Sizing Results']['FuelUsedLb'] = float('nan')
        m['MaxLoad'] = maxLoad
        m['TotalTime'] = totaltime
        m['NumSegments'] = numsegs

    def altitudePowerCurve(self):
        v = self.vconfig
        v['Condition']['Weight'] = self.GW
        speed = 100.
        powers = []
        altitudes = [0]
        v['Condition']['Density'] = self.density(altitudes[0])
        v['Condition']['Speed'] = speed
        (totalPower, Pinduced, Pprofile, Pparasite) = self.powerReq()
        powers.append(totalPower)
        while (not math.isnan(powers[-1])):
            altitudes.append(altitudes[-1] + 1000)
            v['Condition']['Density'] = self.density(altitudes[-1])
            (totalPower, Pinduced, Pprofile, Pparasite) = self.powerReq()
            powers.append(totalPower)
        print altitudes
        print powers


    def generatePowerCurve(self):
        v = self.vconfig
        v['Condition']['Weight'] = self.GW
        speeds = [0]
        powersSL = []
        powersCruise = []
        parasite = []
        profile = []
        induced = []
        # Find hover power and start out the arrays
        v['Condition']['Density'] = self.density(0) # SL
        v['Condition']['Speed'] = speeds[0]
        (totalPower, Pinduced, Pprofile, Pparasite) = self.powerReq()
        powersSL.append(totalPower) # float('nan')
        induced.append(Pinduced)
        profile.append(Pprofile)
        parasite.append(Pparasite)
        # Do the sea level sweep
        while (not math.isnan(powersSL[-1])) or speeds[-1]<100:
            speed = speeds[-1] + v['Simulation']['PowerCurveResolution']
            v['Condition']['Speed'] = speed
            v['Condition']['Density'] = self.density(0) # SL
            (totalPower, Pinduced, Pprofile, Pparasite) = self.powerReq()
            powersSL.append(totalPower) # float('nan')
            induced.append(Pinduced)
            profile.append(Pprofile)
            parasite.append(Pparasite)

            #powersCruise.append(float('nan'))
            speeds.append(speed)
            if debug: print speed
        # # Find cruise hover power and start out the array
        # v['Condition']['Density'] = self.density(v['Condition']['CruiseAltitude']) # 10k ft
        # v['Condition']['Speed'] = speeds[0]
        # powersCruise.append(self.powerReq())
        # # Do the cruise sweep
        # i = 1
        # while (not math.isnan(powersCruise[-1])) or i<len(speeds):
        #     speed = speeds[i-1] + v['Simulation']['PowerCurveResolution']
        #     v['Condition']['Speed'] = speed
        #     v['Condition']['Density'] = self.density(v['Condition']['CruiseAltitude']) # 10k ft
        #     power = self.powerReq()
        #     if speed in speeds:
        #         powersCruise[i] = power
        #     else:
        #         speeds.append(speed)
        #         powersSL.append(float('nan'))
        #         powersCruise.append(power)
        #     i += 1
        #     print speed
        # Store the power curves
        v['Power Curve']['Speeds'] = speeds
        #v['Power Curve']['PowersCruise'] = powersCruise
        v['Power Curve']['PowersSL'] = powersSL
        v['Power Curve']['induced'] = induced
        v['Power Curve']['profile'] = profile
        v['Power Curve']['parasite'] = parasite

    def findHoverCeiling(self):
        v = self.vconfig
        HCmax = v['Simulation']['HoverCeilingMax']
        HCmin = v['Simulation']['HoverCeilingMin']
        altitude = (HCmax + HCmin) / 2
        steps = 0
        v['Condition']['Weight'] = self.GW
        v['Condition']['Speed'] = 0.
        while steps<v['Simulation']['MaxSteps'] and (HCmax-HCmin)>v['Simulation']['HoverCeilingTolerance'] :
            v['Condition']['Density'] = self.density(altitude)
            (powerRequired, Pinduced, Pprofile, Pparasite) = self.powerReq()
            powerAvailable = self.powerAvailable(altitude)
            if debug: pvar(locals(), ('HCmin', 'altitude', 'HCmax', 'powerRequired', 'powerAvailable'))
            if math.isnan(powerRequired) or powerRequired>powerAvailable:
                HCmax = altitude
            else:
                HCmin = altitude
            altitude = (HCmax + HCmin) / 2
        v['Performance']['HoverCeiling'] = altitude

    def density(self, altitude):
        # look at costello notes, RotorcraftPerformance, p.6 for better equation
        return 5e-13*altitude**2 - 7e-8*altitude + .0024 # ISA+15C?


    def powerAvailable(self, altitude):
        return self.vconfig['Powerplant']['MCP'] * self.density(altitude) / self.density(0)#(-2e-5*altitude + 1)

    def SFC(self, power):
        """Returns SFC at a given output power."""
        v = self.vconfig
        # TODO:  Insert partial-power SFC
        #return v['Powerplant']['SFC']
        return -0.00001495*power + v['Powerplant']['SFC'] - v['Powerplant']['MCP']*(-0.00001495)
        # return -0.00001495*power + .4904 # right now this is the GE CT7-8 engine, in the S-92.  Put it back to a generic scaling using v['Powerplant']['SFC'] !

    def speedOfSound(self, density):
        # look at costello notes, RotorcraftPerformance, p.7 for better equation
        return 1026. # yeah... replace this with an equation

    def findMaxEndurance(self):
        """Finds and stores the maximum enduranve data.  Must be run after generatePowerCurve()"""
        v = self.vconfig
        speeds = v['Power Curve']['Speeds']
        powers = v['Power Curve']['PowersSL']
        SPEEDmaxe = 0.
        POWERmaxe = 999999.
        pmin = 9999999.
        for i in range(1, len(speeds)):
            if powers[i] < pmin:
                pmin = powers[i]
                SPEEDmaxe = speeds[i]
                POWERmaxe = powers[i]
        if debug:  print speeds
        if debug:  print powers
        fuelweight = 10000. #self.GW - v['Weights']['EmptyWeightFraction']*self.GW - v['Weights']['UsefulLoad']  # REMOVEME # change back to normal
        hourstoempty = fuelweight / (self.SFC(POWERmaxe) * POWERmaxe)
        v['Performance']['SFCatMaxEndurance'] = self.SFC(POWERmaxe)
        v['Performance']['MaxEndurance'] = hourstoempty
        v['Performance']['MaxEnduranceSpeed'] = SPEEDmaxe
        v['Performance']['PowerAtMaxEnduranceSpeed'] = POWERmaxe


    def findMaxRange(self):
        """Finds and stores the speed for max range and that range.  Must be run after generatePowerCurve()"""
        v = self.vconfig
        speeds = v['Power Curve']['Speeds']
        powers = v['Power Curve']['PowersSL']
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
        fuelweight = v['Weights']['MaxAvailableFuelWeight'] # self.GW - v['Weights']['EmptyWeightFraction']*self.GW - v['Weights']['UsefulLoad']  # REMOVEME # change back to normal
        hourstoempty = fuelweight / (self.SFC(POWERmaxr) * POWERmaxr)
        v['Performance']['SFCatMaxRange'] = self.SFC(POWERmaxr)
        v['Performance']['MaxRange'] = hourstoempty * SPEEDmaxr
        v['Performance']['MaxRangeSpeed'] = SPEEDmaxr
        v['Performance']['PowerAtMaxRangeSpeed'] = POWERmaxr

    def findMaxSpeed(self):
        """Finds and stores the maximum speed.  Must be run after generatePowerCurve()"""
        v = self.vconfig
        powerAvailable = self.powerAvailable(0) # v['Condition']['CruiseAltitude']) # REMOVEME # change back to cruise?
        speeds = v['Power Curve']['Speeds']
        powers = v['Power Curve']['PowersSL']
        maxSpeed = 0.
        maxSpeedPower = 0.
        for i in range(len(speeds)):
            if powers[i]<powerAvailable and speeds[i]>maxSpeed:
                maxSpeed = speeds[i]
                maxSpeedPower = powers[i]
        v['Performance']['MaxSpeed'] = maxSpeed
        v['Performance']['PowerAtMaxSpeed'] = maxSpeedPower
        v['Performance']['SFCatMaxSpeed'] = self.SFC(maxSpeedPower)

    def scaleEngine(self):
        """Scales the engine for high hot hover and fast cruise."""
        v = self.vconfig
        altitude = v['Engine Scaling']['RequiredHoverHeight']
        v['Condition']['Weight'] = self.GW
        v['Condition']['Density'] = self.density(altitude)
        v['Condition']['Speed'] = 0 # hover
        (hoverpower, Pinduced, Pprofile, Pparasite) = self.powerReq()
        if math.isnan(hoverpower):
            self.recordTrimFailure()
            hoverpower = 1.
        v['Engine Scaling']['HoverPowerAtAlt'] = hoverpower
        hoverpower = hoverpower * self.density(0) / self.density(altitude) # scale engine to sea level
        v['Engine Scaling']['HoverPower'] = hoverpower
        # hoverpower = 1.

        # v['Condition']['Weight'] = self.GW
        # altitude = 13000.
        # v['Condition']['Density'] = self.density(altitude)
        # v['Condition']['Speed'] = v['Wing']['MaxSpeed'] / 2. # 1/2 of max speed
        # ceilingpower = self.powerReq()
        # if math.isnan(ceilingpower):
        #     self.recordTrimFailure()
        #     ceilingpower = 1.
        # ceilingpower = ceilingpower * self.density(0) / self.density(altitude) # scale engine to sea level
        # v['Engine Scaling']['CeilingPower'] = ceilingpower
        ceilingpower = 1.

        # v['Condition']['Weight'] = self.GW
        # altitude = v['Condition']['CruiseAltitude']
        # v['Condition']['Density'] = self.density(altitude)
        # v['Condition']['Speed'] = v['Wing']['MaxSpeed']
        # cruisepower = self.powerReq()
        # if math.isnan(cruisepower):
        #     self.recordTrimFailure()
        #     cruisepower = 1.
        # cruisepower = cruisepower * self.density(0) / self.density(altitude) # scale engine to sea level
        if 'Speeds' in v['Power Curve']:
            cruisepower = v['Power Curve']['PowersSL'][-2]
        else:
            cruisepower = 1.
        v['Engine Scaling']['CruisePower'] = cruisepower

        power = max(hoverpower, ceilingpower, cruisepower)
        # power = 7300  # REMOVEME - reinsert proper engine sizing
        gamma = power / self.vconfig['Powerplant']['BaselineMRP']
        sfc = (-0.00932*gamma**2+0.865*gamma+0.445)/(gamma+0.301)*v['Powerplant']['BaselineSFC']
        v['Powerplant']['MRP'] = power * 1.3
        v['Powerplant']['MCP'] = power
        v['Powerplant']['SFC'] = sfc * (1-v['Powerplant']['SFCTechImprovementFactor'])

    def flyMission(self):
        m = self.mconfig
        v = self.vconfig
        v['Sizing Results']['MisSize'] = float('nan')
        if v['Weights']['EmptyWeightFraction']>1 or v['Weights']['MaxAvailableFuelWeight']<0 or not v['Sizing Results']['CouldTrim'] :
            return
        elapsed = 0 # elapsed time since mission start
        fuelAvailable = self.GW - v['Weights']['EmptyWeight'] - v['Weights']['UsefulLoad'] # total fuel weight available, pounds
        w = v['Weights']['EmptyWeight'] + fuelAvailable
        totalFuel = 0.
        # step through all the segments specified in the mission file
        for i in range(0, m['NumSegments']):
            seg = 'Segment %d' % (i+1)
            # Copy the segment data into the Condition section
            def makeCurrent(section, key, vconfig):
                vconfig['Condition'][key] = section[key]
            m[seg].walk(makeCurrent, vconfig=v)
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
                (power, Pinduced, Pprofile, Pparasite) = self.powerReq()
                if math.isnan(power):
                    self.recordTrimFailure()
                    return
                fuel = self.SFC(power) * power * (duration/60)
                w -= fuel
                totalFuel += fuel
                elapsed += duration
            if debug: print('%s RfR: %.3f' % (seg, totalFuel/self.GW))
        self.misSize = fuelAvailable - totalFuel
        if debug: print 'Finished!  Total fuel used: %s     Missize amount: %s' % (totalFuel, self.misSize)
        v['Sizing Results']['FuelUsedLb'] = totalFuel
        v['Sizing Results']['Payload'] = m['Segment 2']['PayloadWeight']
        v['Sizing Results']['FuelUsedGal'] = totalFuel / 6.83
        v['Sizing Results']['TotalWeight'] = v['Weights']['EmptyWeight'] + m['MaxLoad'] + totalFuel
        v['Sizing Results']['MisSize'] = self.misSize

    def recordTrimFailure(self):
        v = self.vconfig
        v['Sizing Results']['CouldTrim'] = False
        def recordFailure(section, key, vconfig):
            vconfig['Trim Failure'][key] = section[key]
        v['Condition'].walk(recordFailure, vconfig=v)

    def powerReq(self):
        v = self.vconfig

        # REMOVEME
        if v['Main Rotor']['NumRotors'] > 1:
            advancingLiftBalance = .9
            if v['Wing']['SpanRadiusRatio'] > 1:
                advancingLiftBalance = .95
        elif v['Wing']['SpanRadiusRatio'] > 1:
            advancingLiftBalance = .8
        else:
            advancingLiftBalance = .6
        if v['Condition']['Speed'] < 50:
            self.rotor.Vtip = v['Main Rotor']['TipSpeed']
            self.rotor.omega = self.rotor.Vtip / v['Main Rotor']['Radius']
            #v['Main Rotor']['TipSpeed'] = self.rotor.Vtip
        else:
            self.rotor.Vtip = v['Main Rotor']['SlowedTipSpeed']
            self.rotor.omega = self.rotor.Vtip / v['Main Rotor']['Radius']
            #v['Main Rotor']['TipSpeed'] = self.rotor.Vtip

        Density = v['Condition']['Density']
        V = v['Condition']['Speed'] * 1.687 # speed in feet per second

        # proportion out the vertical lift between the rotors and the wing
        VerticalLift_wing_max = 0.5 * v['Wing']['WingArea'] * Density * V**2 * v['Wing']['WingClMax'] # wing produces as much lift as it can
        VerticalLift_wing = min(v['Wing']['MaxWingLoadProportion']*v['Condition']['Weight'], VerticalLift_wing_max) # ...up to some  of the weight
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
        # WingDrag = 0.

        # proportion out forward thrust between the aux prop and the rotors
        BodyDrag = .5 * Density * V**2 * v['Body']['FlatPlateDrag']
        TotalDrag = BodyDrag + WingDrag
        ForwardThrust = TotalDrag
        # v['Aux Propulsion']['NumAuxProps'] = 1
        if v['Aux Propulsion']['NumAuxProps'] > 0:
            ForwardThrust_auxprops = ForwardThrust * v['Aux Propulsion']['PropulsionFactor']
            ForwardThrust_perAuxprop = ForwardThrust_auxprops / v['Aux Propulsion']['NumAuxProps']
        else:
            ForwardThrust_auxprops = 0.
            ForwardThrust_perAuxprop = 0.
        ForwardThrust_rotors = ForwardThrust - ForwardThrust_auxprops
        ForwardThrust_perRotor = ForwardThrust_rotors / v['Main Rotor']['NumRotors']
        if debug: pvar(locals(), ('ForwardThrust_perRotor', 'VerticalLift_perRotor', 'VerticalLift_wing'))

        # calculate rotor power
        (singleRotorPower, Pinduced, Pprofile) = self.rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], V=V, rho=Density, speedOfSound=self.speedOfSound(Density), Fx=ForwardThrust_perRotor, Fz=VerticalLift_perRotor, maxSteps=v['Simulation']['MaxSteps'], advancingLiftBalance=advancingLiftBalance, returnAll=True)
        # while math.isnan(singleRotorPower) and advancingLiftBalance<1:
        #     advancingLiftBalance += .1
        #     singleRotorPower = self.rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], V=V, rho=Density, speedOfSound=self.speedOfSound(Density), Fx=ForwardThrust_perRotor, Fz=VerticalLift_perRotor, maxSteps=v['Simulation']['MaxSteps'], advancingLiftBalance=advancingLiftBalance)
        singleRotorPower = singleRotorPower / (1-v['Body']['DownwashFactor']) * v['Main Rotor']['Kint']  # REMOVEME # fix Kint?

        # calculate prop power
        if ForwardThrust_perAuxprop>0:
            # singlePropPower = self.rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], V=V, rho=Density, speedOfSound=self.speedOfSound(Density), Fx=ForwardThrust_perAuxprop, Fz=.1, maxSteps=v['Simulation']['MaxSteps'], advancingLiftBalance=.5)
            singlePropPower = V*0.19458 + 73.7097 # REMOVEME
        else:
            singlePropPower = 0.

        wingPower = WingDrag*V/550

        if debug: pvar(locals(), ('BodyDrag', 'WingDrag', 'VerticalLift_perRotor', 'VerticalLift_wing', 'singleRotorPower', 'singlePropPower', 'wingPower'))
        # find total power
        totalPower = singleRotorPower*v['Main Rotor']['NumRotors'] + singlePropPower*v['Aux Propulsion']['NumAuxProps'] + TotalDrag*V/550 # Is this right?  should the parasite power be just added on directly like this?
        totalPower = totalPower / (1-v['Antitorque']['AntitorquePowerFactor']) / (1-v['Powerplant']['TransmissionEfficiency'])

        v['Performance']['MaxBladeLoadingSeen'] = max(v['Performance']['MaxBladeLoadingSeen'], math.sqrt(ForwardThrust_perRotor**2+VerticalLift_perRotor**2)/(Density*v['Main Rotor']['DiskArea']*self.rotor.Vtip**2))

        Pparasite = TotalDrag*V/550

        # Pparasite = singlePropPower

        return (totalPower, Pinduced, Pprofile, Pparasite)


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
    import matplotlib.pylab as pylab
    import numpy as np
    v = ConfigObj(vehicleConfigPath, configspec='Config/vehicle.configspec')
    m = ConfigObj(missionConfigPath, configspec='Config/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    c81File_mainRotor = 'Config/%s'%v['Main Rotor']['AirfoilFile']
    c81File_auxProp = 'Config/%s'%v['Aux Propulsion']['AirfoilFile']
    airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
    airfoildata_auxProp = np.genfromtxt(c81File_auxProp, skip_header=0, skip_footer=0) # read in the airfoil file
    plt.figure(num=None, figsize=(6, 4), facecolor='w', edgecolor='k')
    for GW in [24250., 28660., 33069.]:
        vehicle = Vehicle(v, m, GW, airfoildata_mainRotor, airfoildata_auxProp)
        vehicle.flyMission()
        vehicle.generatePowerCurve()
        # vehicle.findHoverCeiling()
        vehicle.findMaxRange()
        vehicle.findMaxEndurance()
        vehicle.findMaxSpeed()
        vehicle.altitudePowerCurve()
        vehicle.write()

        plt.plot(vehicle.vconfig['Power Curve']['Speeds'], vehicle.vconfig['Power Curve']['PowersSL'])
    # plt.plot(vehicle.vconfig['Power Curve']['Speeds'], vehicle.vconfig['Power Curve']['PowersCruise'])
    MCPspeeds = [0, 300]
    MCPSL = [vehicle.powerAvailable(0), vehicle.powerAvailable(0)]
    # MCPalt = [vehicle.powerAvailable(vehicle.vconfig['Condition']['CruiseAltitude']), vehicle.powerAvailable(vehicle.vconfig['Condition']['CruiseAltitude'])]
    plt.plot(MCPspeeds, MCPSL)
    # plt.plot(MCPspeeds, MCPalt, 'g')
    plt.title('Sea Level')
    plt.legend(('11000 kg', '13000 kg', '15000 kg', 'MCP'), loc=4)
    plt.axis([0, 260, 0, 8000])
    plt.xlabel('Speed (kts)')
    plt.ylabel('Power (hp)')
    plt.tight_layout()
    plt.grid(True)
    pylab.savefig('Output/Figures/PowerCurveCruise.png', bbox_inches=0, dpi=600)
    plt.show()