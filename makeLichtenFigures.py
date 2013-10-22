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
import numpy as np
import scipy.ndimage
import matplotlib.pylab as pylab
import matplotlib.pyplot as plt
from vehicle import Vehicle
from configobj import ConfigObj
from validate import Validator
import random
from scipy.interpolate import interp1d


# BEGIN SCRIPT CONFIG BLOCK
# plot display parameters
singlefigW = 3.4
singlefigH = 2.5 # * 2
doublefigW = 7
doublefigH = 5
figDPI = 600  # change back to 600?
inline_spacing = 5
contourfontsize = 6
titlefontsize = 10
labelfontsize = 8
labelboxcolor = 'gray'
labeltextcolor = ''
labelalpha = .8
labelpad = 5
labelHoffset = -20
labelVoffset = 50
linewidth = 3
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

# script behavior
displayPlots = False
saveData = False
saveFigures = True
annotatePlots = True

# plot selection
plotScalingPlots = False
plotRfPlot = False
plotPerformanceCurve = False
plotcross_config_plots = True

# config files
vehicleConfigPath = 'Config/vehicle.cfg'
missionConfigPath = 'Config/mission_singlesegment.cfg'
# END SCRIPT CONFIG BLOCK





vconfig = ConfigObj(vehicleConfigPath, configspec='Config/vehicle.configspec')
mconfig = ConfigObj(missionConfigPath, configspec='Config/mission.configspec')
vvdt = Validator()
vconfig.validate(vvdt)
mvdt = Validator()
mconfig.validate(mvdt)
c81File='Config/%s'%vconfig['Main Rotor']['AirfoilFile']
airfoildata = np.genfromtxt(c81File, skip_header=0, skip_footer=0) # read in the airfoil file


def cross_config_plots():

    cost = np.array([0.5, 1, 1.5, 2, 3, 3.5, 4])


    data = [[[5, 7, 7.4, 7.6, 8.2, 8.5], [1, 1.6, 2.2, 2.9, 4], [2.5, 2.8, 3.1, 3.3, 3.4], [1, 4, 4.7, 4.84, 5.03]],
        [[2.5, 6, 7.8, 8.3, 9, 9.6], [0.5, 1.2, 2, 2.7, 4], [1.5, 2.6, 3, 3.1, 3.2], [0.2, 3, 4, 4.3, 4.5]],
        [[4.2, 6.5, 8, 8.5, 9, 9.5, 9.9], [0.5, 1.3, 2.2, 3.2, 4.4, 4.5, 4.6], [2.5, 3, 3.3, 3.55, 4], [0.1, 3, 4.1, 4.2]],
        [[4, 6.5, 8, 8.8, 10, 10.5, 10.8], [0.7, 1.5, 2.5, 3.5, 4.5, 4.7], [3, 3.8, 4.2, 4.4, 4.5, 4.5], [0, 3, 4, 4.2, 4.25]],
        [[3.5, 6, 7.5, 8, 8.5, 9, 9.4], [0.3, 0.7, 1.2, 1.7, 3, 4, 4.3], [1.5, 2, 2.4, 2.7, 3.1, 3.2], [0.5, 3, 4.9, 5.3]],
        [[3, 5, 6.5, 7.5, 9.5, 10.8, 11.4], [0.1, 0.6, 1.2, 1.8, 3.2, 4.2, 4.5], [1.5, 2, 2.3, 2.5, 2.8, 2.8, 2.8], [0.2, 2.5, 4.5, 4.9]]]

    # originals:
    # conventional = np.array([[5, 7, 7.2, 7.5, 8.2, 8.5, 7.5], [1, 1.5, 2.1, 3, 4, 3.4, 3], [2.5, 2.8, 3.1, 3.2, 3.2, 2.8, 2.5], [1, 4, 4.5, 4.8, 5, 4.5, 4]])
    # prop = np.array([[2.5, 6, 7.8, 8.3, 9, 9.5, 8], [0.5, 1.2, 2, 2.7, 4, 3, 2], [1.5, 2.6, 3, 3.1, 3.2, 2.5, 2], [0.2, 4, 4.5, 4.5, 4.5, 4, 3.5]])
    # wing = np.array([[4.2, 6.5, 8, 8.5, 9, 9.5, 9.5], [0.5, 1.4, 2.2, 3.2, 4.5, 4.5, 4.5], [2.5, 3, 3.3, 3.55, 4, 3.8, 3], [0.1, 4, 4.5, 5, 5, 5, 4]])
    # propwing = np.array([[4, 6.5, 8, 8.5, 10, 10.5, 10.6], [0.7, 1.2, 2.5, 3.5, 4.5, 4.5, 4], [3, 3.8, 4.2, 4.3, 4.5, 4.5, 4], [0, 4, 4.2, 4.3, 4.3, 4, 3]])
    # coax = np.array([[3.5, 5, 7.5, 8, 8.5, 9, 9], [0.3, 0.7, 1.2, 1.7, 3, 4, 4], [1.5, 2, 2.4, 2.7, 3.1, 3.2, 3], [0.5, 2, 5, 5, 4.8, 4.5, 4.2]])
    # coaxprop = np.array([[3, 5, 6.5, 7.5, 9.5, 10.8, 11], [0.1, 0.7, 1.2, 2, 3.2, 4.2, 4.4], [1.5, 2, 2.2, 2.4, 2.8, 2.8, 2.8], [0.3, 1.5, 4.5, 4.8, 4.5, 4.5, 4]])

    labels = ['Mission Capability Index', '500nm Payload', 'Max Range', 'Hover Ceiling']

    first = True
    plt.figure(num=None, figsize=(doublefigW, doublefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    for i in xrange(4):
        plt.subplot(2, 2, i)
        for configuration in data:
            f2 = interp1d(cost[:len(configuration[i])], configuration[i], kind='cubic')
            y = np.arange(.5, cost[:len(configuration[i])][-1], .1)
            x = f2(y)
            plt.plot(x, y, linewidth=linewidth)
        if first:
            plt.legend(('SMR', 'SMR+prop', 'SMR+wing', 'SMR+prop+wing', 'Coax', 'Coax+prop'))
            plt.tick_params(color='#FFFFFF', labelcolor='#FFFFFF')
            first = False
        plt.tick_params(labelsize=axislabelfontsize)
        plt.xlabel(labels[i])
        plt.ylabel('Normalized Base Cost')
        plt.axis([0, 6, .5, 4])
    plt.tight_layout()
    if saveFigures: pylab.savefig('Output/Figures/MCI_components.png', dpi=figDPI, bbox_inches=0)
    if displayPlots: plt.show()

    plt.figure(num=None, figsize=(doublefigW, doublefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    i = 0
    for configuration in data:
        f2 = interp1d(cost[:len(configuration[i])], configuration[i], kind='cubic')
        y = np.arange(.5, cost[:len(configuration[i])][-1], .1)
        x = f2(y)
        plt.plot(x, y, linewidth=linewidth)
    plt.legend(('SMR', 'SMR+prop', 'SMR+wing', 'SMR+prop+wing', 'Coax', 'Coax+prop'), loc=2)
    plt.xlabel(labels[i])
    plt.ylabel('Normalized Base Cost')
    plt.axis([0, 12, .5, 4])

    plt.tight_layout()
    if saveFigures: pylab.savefig('Output/Figures/MCI.png', dpi=figDPI, bbox_inches=0)
    if displayPlots: plt.show()


def RfPlot():
    plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    GW = np.arange(15000, 40000, 100)
    RfR = -5e-10*GW**2 + 5e-5*GW-0.9699 #2e-14*GW**3 - 2e-9*GW**2 + 0.0001*GW - 1.4685
    RfA = 1e-11*GW**2 - 2e-6*GW + 0.0335
    for i in xrange(GW.size):
        if RfR[i]>RfA[i]:
            intX = GW[i]
            intY = RfA[i]
            break
    plt.plot(GW, RfR)
    plt.plot(GW, RfA)
    print intX
    plt.plot([intX, intX], [-.4, intY], 'k:')
    plt.plot([15000, intX], [intY, intY], 'k:')
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xticks([])
    plt.yticks([])
    plt.xlabel(r'Gross Weight', fontsize=labelfontsize)
    plt.ylabel(r'$R_f$', fontsize=labelfontsize)
    #plt.title(r'Nominal $R_f$ graphical plot', fontsize=titlefontsize)
    plt.legend((r'$R_{f_R}$', r'$R_{f_A}$'), fontsize=labelfontsize)
    plt.text(intX+600, -.2, 'Minimum GW\nfor mission', fontsize=labelfontsize)

    plt.tight_layout()

    if saveFigures: pylab.savefig('Output/Figures/RfPlot.png', dpi=figDPI, bbox_inches=0)
    if displayPlots: plt.show()

def density(altitude):
    return 5e-13*altitude**2 - 7e-8*altitude + .0024 # ISA+15C?


def PerformanceCurve():
    vconfig = ConfigObj('Config/vehicle_s92.cfg', configspec='Config/vehicle.configspec')
    vconfig.validate(vvdt)
    v = ConfigObj(vconfig)
    m = ConfigObj(mconfig)
    c81File_mainRotor = 'Config/%s'%vconfig['Main Rotor']['AirfoilFile']
    airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
    c81File_auxProp = 'Config/%s'%vconfig['Aux Propulsion']['AirfoilFile']
    airfoildata_auxProp = np.genfromtxt(c81File_auxProp, skip_header=0, skip_footer=0) # read in the airfoil file
    s92 = Vehicle(v, m, 26000, airfoildata_mainRotor, airfoildata_auxProp)
    s92.generatePowerCurve()
    s92.scaleEngine()
    s92.findHoverCeiling()
    s92.findMaxRange()
    s92.findMaxSpeed()
    s92.findMaxEndurance()
    s92.OEC()
    #s92.write()
    s92_V = np.array([50.1073, 60.2146, 69.9122, 80.0195, 90.1268, 100.098, 110.205, 120.312, 127.415, 137.249, 144.624, 151.59, 156.644, 163.2, 167.98])
    s92_fuel = np.array([1240.51, 1156.96, 1113.92, 1096.2, 1103.8, 1129.11, 1177.22, 1240.51, 1301.27, 1410.13, 1518.99, 1655.7, 1772.15, 1972.15, 2134.18])
    s92_HP = 6.68896*(2452-np.sqrt(6012300-1495*s92_fuel))

    powerSL = np.array(s92.vconfig['Power Curve']['PowersSL']) + 300
    sfcSL = -0.00001495*powerSL + .4904
    fuelSL = sfcSL * powerSL

    plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    plt.subplot(221)
    plt.plot(s92.vconfig['Power Curve']['Speeds'], powerSL)
    plt.plot(s92_V, s92_HP, marker='o', markersize=3, linestyle='')
    plt.axis([0, 180, 1500, 5000])
    #plt.legend(('Predicted', 'Published', 'parasite', 'induced', 'profile'), fontsize=labelfontsize)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xlabel('Airspeed (kts)', fontsize=labelfontsize)
    plt.ylabel('HP', fontsize=labelfontsize)
    plt.title('S-92', fontsize=titlefontsize)
    plt.locator_params(nbins=6)
    plt.tight_layout()
    plt.grid(True)

    plt.subplot(222)
    vconfig = ConfigObj('Config/vehicle_xh59.cfg', configspec='Config/vehicle.configspec')
    vconfig.validate(vvdt)
    v = ConfigObj(vconfig)
    m = ConfigObj(mconfig)
    c81File_mainRotor = 'Config/%s'%vconfig['Main Rotor']['AirfoilFile']
    airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
    c81File_auxProp = 'Config/%s'%vconfig['Aux Propulsion']['AirfoilFile']
    airfoildata_auxProp = np.genfromtxt(c81File_auxProp, skip_header=0, skip_footer=0) # read in the airfoil file
    xh59 = Vehicle(v, m, 12500., airfoildata_mainRotor, airfoildata_auxProp)
    xh59.generatePowerCurve()
    xh59.findMaxRange()
    xh59.findMaxSpeed()
    # xh59.write()
    plt.plot(xh59.vconfig['Power Curve']['Speeds'], xh59.vconfig['Power Curve']['PowersSL'])
    xh59_V = np.array([0, 21.966997, 42.561056, 63.471947, 85.122112, 87.762376, 89.663366, 91.669967, 105.927393, 108.567657, 120.712871, 125.887789, 126.627063, 130.112211, 136.765677, 136.554455, 143.841584, 146.481848, 151.339934, 154.719472])
    xh59_HP = np.array([1414.285714, 1168.163265, 925.714286, 813.673469, 732.857143, 800.816327, 854.081633, 786.122449, 879.795918, 817.346939, 912.857143, 1129.591837, 1050.612245, 1002.857143, 1054.285714, 1206.734694, 1307.755102, 1434.489796, 1520.816327, 1647.55102])
    plt.plot(xh59_V, xh59_HP, marker='o', markersize=3, linestyle='')
    plt.axis([0, 180, 500, 2000])
    #plt.legend(('Predicted', 'Published'), fontsize=labelfontsize)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xlabel('Airspeed (kts)', fontsize=labelfontsize)
    plt.ylabel('HP', fontsize=labelfontsize)
    plt.title('XH-59', fontsize=titlefontsize)
    plt.locator_params(nbins=6)
    plt.tight_layout()
    plt.grid(True)

    plt.subplot(223)
    vconfig = ConfigObj('Config/vehicle_ch47.cfg', configspec='Config/vehicle.configspec')
    vconfig.validate(vvdt)
    v = ConfigObj(vconfig)
    m = ConfigObj(mconfig)
    c81File_mainRotor = 'Config/%s'%vconfig['Main Rotor']['AirfoilFile']
    airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
    c81File_auxProp = 'Config/%s'%vconfig['Aux Propulsion']['AirfoilFile']
    airfoildata_auxProp = np.genfromtxt(c81File_auxProp, skip_header=0, skip_footer=0) # read in the airfoil file
    ch47 = Vehicle(v, m, 54000., airfoildata_mainRotor, airfoildata_auxProp)
    ch47.generatePowerCurve()
    ch47.findMaxRange()
    ch47.findMaxSpeed()
    #ch47.write()
    plt.plot(ch47.vconfig['Power Curve']['Speeds'], ch47.vconfig['Power Curve']['PowersSL'])

    ch_47_V = np.array([59.966790, 69.955234, 79.920862, 90.124157, 100.039590, 110.188126, 119.783755, 129.870688])
    ch_47_HP = np.array([4169.052997, 3951.635574, 3840.705790, 3841.611245, 3964.954268, 4221.430057, 4616.293344, 5160.285760])

    ch_47_V_alt = np.array([49.964656, 53.389711, 57.343716, 62.343072, 67.859969, 72.585153, 78.612748, 85.682271, 93.265914, 100.57766, 105.26862, 109.17357, 113.33558, 117.49531, 120.08416, 123.46016, 127.34116, 129.923165  ])
    ch_47_HP_alt = np.array([4450.363004, 4338.852801, 4200.767121, 4068.098691, 3962.098604, 3887.975158, 3840.589706, 3814.594649, 3831.241082, 3895.783736, 3981.391749, 4072.254495, 4184.437985, 4307.270239, 4435.287574, 4552.725795, 4755.400562, 4915.364189])

    plt.plot(ch_47_V, ch_47_HP, marker='o', markersize=3, linestyle='')
    plt.axis([40, 140, 3000, 6000])
    #plt.legend(('Predicted', 'Published'), fontsize=labelfontsize)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xlabel('Airspeed (kts)', fontsize=labelfontsize)
    plt.ylabel('HP', fontsize=labelfontsize)
    plt.title('CH-47', fontsize=titlefontsize)
    plt.locator_params(nbins=6)
    plt.tight_layout()
    plt.grid(True)

    plt.subplot(224)
    vconfig = ConfigObj('Config/vehicle_ch47-347.cfg', configspec='Config/vehicle.configspec')
    vconfig.validate(vvdt)
    v = ConfigObj(vconfig)
    m = ConfigObj(mconfig)
    c81File_mainRotor = 'Config/%s'%vconfig['Main Rotor']['AirfoilFile']
    airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
    c81File_auxProp = 'Config/%s'%vconfig['Aux Propulsion']['AirfoilFile']
    airfoildata_auxProp = np.genfromtxt(c81File_auxProp, skip_header=0, skip_footer=0) # read in the airfoil file
    ch47_347 = Vehicle(v, m, 45000., airfoildata_mainRotor, airfoildata_auxProp)
    ch47_347.generatePowerCurve()
    ch47_347.findMaxRange()
    ch47_347.findMaxSpeed()
    #ch47_347.write()
    plt.plot(ch47_347.vconfig['Power Curve']['Speeds'], ch47_347.vconfig['Power Curve']['PowersSL'])


    ch_47_347_V = np.array([49.722760, 56.252532, 62.187081, 68.241735, 75.842437, 84.277983, 91.408871, 97.472955, 107.11309, 115.45151, 122.36576, 130.12487, 136.81718, 142.67818, 146.74880, 150.94070, 154.53679, 157.05606])
    ch_47_347_HP = np.array([4643.003656, 4526.202972, 4403.491978, 4292.871891, 4181.861387, 4094.882537, 4044.596857, 4030.944283, 4101.237321, 4238.525876, 4406.477164, 4701.488089, 5033.132119, 5377.107312, 5654.867821, 5956.840177, 6246.841754, 6470.448454])


    ch_47_347_V_alt = np.array([49.477716, 59.562914, 70.250995, 79.879345, 90.226558, 99.987388, 110.46885, 120.00715, 131.22633, 140.30601, 149.87614, 160.42066])
    ch_47_347_HP_alt = np.array([3346.093204, 3101.121697, 2940.846604, 2889.930251, 2893.377931, 2981.822345, 3142.812202, 3388.888889, 3798.177805, 4220.128237, 4793.470280, 5602.930378])

    #ch_47_347_V = np.array([69.999725, 79.976762, 90.191464, 100.38905, 110.04058, 119.93548, 130.06576, 140.162965])
    #ch_47_347_HP = np.array([3743.984677, 3579.811073, 3527.472708, 3555.000073, 3688.968643, 3908.150541, 4249.816442, 4745.889420])

    # ch_47_347_V_alt = np.array([60.302565, 67.650823, 74.206227, 84.163869, 93.846199, 103.51369, 112.12101, 117.32609, 123.04643, 127.72028, 132.90939, 136.53675, 140.42002, 144.048522  ])
    # ch_47_347_HP_alt = np.array([3822.991386, 3717.153816, 3648.517270, 3574.858160, 3565.068417, 3624.495640, 3747.722579, 3875.972082, 4041.538692, 4207.012435, 4409.803286, 4575.184162, 4767.210165, 4927.266659])

    plt.plot(ch_47_347_V_alt, ch_47_347_HP_alt, marker='o', markersize=3, linestyle='')
    plt.axis([40, 180, 2000, 6000])
    #plt.legend(('Predicted', 'Published'), fontsize=labelfontsize)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xlabel('Airspeed (kts)', fontsize=labelfontsize)
    plt.ylabel('HP', fontsize=labelfontsize)
    plt.title('Boeing Model 347', fontsize=titlefontsize)
    plt.locator_params(nbins=6)
    plt.tight_layout()
    plt.grid(True)


    if saveFigures: pylab.savefig('Output/Figures/S92PerformanceCurve.png', bbox_inches=0, dpi=figDPI)
    if displayPlots: plt.show()


def ScalingPlots():
    # flat plate drag area
    plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    dragGW = np.arange(0, 40000, 100)
    flatplatedrag = 0.25 * dragGW**.5
    heliDrags = np.array([15, 21, 22.5, 37, 32, 38, 41, 43, 56, 40])
    heliGWs = np.array([2.2, 8, 9, 13, 18, 20.5, 22, 33, 39.5, 25]) * 1000
    heliLabels = np.array(['S-52', 'S-55', 'UH-1', 'S-58', 'S-61', 'CH-46', 'SA 321', 'CH-47', 'CH-53A', 'S-92'])
    heliLabelsX = np.array([500, 8000, 10000, 9000, 19000, 15000, 20000, 33000, 33000, 25000])
    heliLabelsY = np.array([17,  16,   21,    39,   27,    39,    43,    38,    51,    35])
    print heliGWs-2000
    print heliDrags+2
    for i in xrange(len(heliLabels)):
        plt.text(heliLabelsX[i], heliLabelsY[i], heliLabels[i], fontsize=labelfontsize)#, bbox={'facecolor':labelboxcolor, 'edgecolor':labelboxcolor, 'alpha':labelalpha, 'pad':labelpad})
    plt.plot(heliGWs, heliDrags, 'ko')
    plot = plt.plot(dragGW, flatplatedrag)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xlabel('Gross Weight (lbs)', fontsize=labelfontsize)
    plt.ylabel('Flat Plate Drag Area (sq ft)', fontsize=labelfontsize)
    #plt.title('Flat Plate Drag Scaling', fontsize=titlefontsize)
    plt.xticks(np.arange(0, 40000, 10000))
    plt.tight_layout()
    if saveFigures: pylab.savefig('Output/Figures/flatPlateDragScaling.png', bbox_inches=0, dpi=figDPI)
    if displayPlots: plt.show()

    # engine weight
    plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    MCP = np.arange(0, 4000, 10)
    weight = ((0.1054*(MCP)**2+358*(MCP)+2.757*10**4)/((MCP)+1180))
    plot = plt.plot(MCP, weight)
    #plt.text(200, 580, r'$\frac{0.1054*MCP^2 + 358*MCP + 2.757*10^4}{MCP+1180}$', fontsize=labelfontsize+4)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xticks(np.arange(0, 5000, 1000))
    plt.xlabel('Max Continuous Power (hp)', fontsize=labelfontsize)
    plt.ylabel('Engine Weight (lbs)', fontsize=labelfontsize)
    #plt.title('Engine Weight Scaling', fontsize=titlefontsize)
    plt.tight_layout()
    if saveFigures: pylab.savefig('Output/Figures/EngineWeight_Scaling.png', bbox_inches=0, dpi=figDPI)
    if displayPlots: plt.show()

    # drive system weight
    plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    GW = np.arange(0, 40000, 100)
    MCP = [2000., 4000., 6000.]
    weight = np.zeros((len(MCP), GW.size))
    DL = 10.
    for i in range(len(MCP)):
        weight[i] = (525.*(GW/1000.)**1.14)/((GW/MCP[i])**0.763*DL**0.381)
        plot = plt.plot(GW, weight[i])
        plt.text(30000, weight[i][300]-200, '%d hp' % MCP[i], fontsize=labelfontsize)
    #plt.text(15000, 400, r'$\frac{525*(GW/1000)^{1.14}}{(GW/MCP)^{.763}*DL^{.381}}$', fontsize=labelfontsize+5)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xticks(np.arange(0, 50000, 10000))
    plt.xlabel('Gross Weight (lbs)', fontsize=labelfontsize)
    plt.ylabel('Drive System Weight (lbs)', fontsize=labelfontsize)
    #plt.title('Drive System Scaling (DL=10psf)', fontsize=titlefontsize)
    plt.tight_layout()
    if saveFigures: pylab.savefig('Output/Figures/DriveSystemWeight_Scaling.png', bbox_inches=0, dpi=figDPI)
    if displayPlots: plt.show()

    # wing weight
    plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    GW = np.arange(0, 40000, 100)
    SRR = [1., 2., 3.]
    weights = np.zeros((len(SRR), GW.size))
    DL = 10
    labelX = [31000, 31000, 30000]
    labelY = [600, 1200, 1800]
    for i in xrange(len(SRR)):
        weights[i] = 0.00272*GW**1.4/DL**0.8*SRR[i]**0.8
        plt.plot(GW, weights[i])
        plt.text(labelX[i], labelY[i], r'$\frac{Span}{R}=%d$' % SRR[i], fontsize=labelfontsize)
    #plt.text(5000, 2250, r'$\frac{.00272*GW^{1.4}}{DL^{.8}}*\frac{Span}{R}^{.8}$', fontsize=labelfontsize+5)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xticks(np.arange(0, 50000, 10000))
    plt.xlabel('Gross Weight (lbs)', fontsize=labelfontsize)
    plt.ylabel('Wing Weight (lbs)', fontsize=labelfontsize)
    #plt.title('Wing Weight Scaling (DL=10psf)', fontsize=titlefontsize)
    plt.tight_layout()
    if saveFigures: pylab.savefig('Output/Figures/WingWeight_Scaling.png', bbox_inches=0, dpi=figDPI)
    if displayPlots: plt.show()

    # SFC
    plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    gamma = np.arange(.5, 4, .1)
    SFCfrac = (-0.00932*gamma**2+0.865*gamma+0.445)/(gamma+0.301)
    plt.plot(gamma, SFCfrac)
    #plt.text(.8, 1.05, r'$\frac{SFC}{SFC_{baseline}}=\frac{-.00932*\gamma^2+.865*\gamma+.445}{\gamma+.301}$', fontsize=labelfontsize+4)
    plt.tick_params(labelsize=axislabelfontsize)
    #plt.xticks(np.arange(0, 50000, 10000))
    plt.xlabel(r'$\gamma = MCP/MCP_{baseline}$', fontsize=labelfontsize)
    plt.ylabel(r'$SFC/SFC_{baseline}$', fontsize=labelfontsize)
    #plt.title('Engine SFC Scaling', fontsize=titlefontsize)
    plt.tight_layout()
    if saveFigures: pylab.savefig('Output/Figures/SFC_Scaling.png', bbox_inches=0, dpi=figDPI)
    if displayPlots: plt.show()



if __name__ == '__main__':
    if plotScalingPlots: ScalingPlots()
    if plotRfPlot: RfPlot()
    if plotPerformanceCurve: PerformanceCurve()
    if plotcross_config_plots: cross_config_plots()