import math
import numpy as np
import scipy.ndimage
import matplotlib.pylab as pylab
import matplotlib.pyplot as plt
from vehicle import Vehicle
from configobj import ConfigObj
from validate import Validator
import random

singlefigW = 3.4
singlefigH = 2.5 * 2
doublefigW = 7
doublefigH = 5
figDPI = 300  # change back to 600?
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




displayPlots = True
saveData = False
saveFigures = False
annotatePlots = True

plotScalingPlots = False
plotRfPlot = False
plotPerformanceCurve = True








vconfig = ConfigObj('Config/vehicle.cfg', configspec='Config/vehicle.configspec')
mconfig = ConfigObj('Config/mission_singlesegment.cfg', configspec='Config/mission.configspec')
vvdt = Validator()
vconfig.validate(vvdt)
mvdt = Validator()
mconfig.validate(mvdt)
c81File='Config/%s'%vconfig['Main Rotor']['AirfoilFile']
airfoildata = np.genfromtxt(c81File, skip_header=0, skip_footer=0) # read in the airfoil file


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
    s92.write()
    # v['Simulation']['PowerCurveResolution'] = 1
    s92_V = np.array([50.1073, 60.2146, 69.9122, 80.0195, 90.1268, 100.098, 110.205, 120.312, 127.415, 137.249, 144.624, 151.59, 156.644, 163.2, 167.98])
    s92_fuel = np.array([1240.51, 1156.96, 1113.92, 1096.2, 1103.8, 1129.11, 1177.22, 1240.51, 1301.27, 1410.13, 1518.99, 1655.7, 1772.15, 1972.15, 2134.18])
    s92_HP = 6.68896*(2452-np.sqrt(6012300-1495*s92_fuel))

    powerSL = np.array(s92.vconfig['Power Curve']['PowersSL'])
    sfcSL = -0.00001495*powerSL + .4904
    fuelSL = sfcSL * powerSL

    plt.figure(num=None, figsize=(singlefigW, singlefigH), dpi=figDPI, facecolor='w', edgecolor='k')
    plt.subplot(211)
    plt.plot(s92.vconfig['Power Curve']['Speeds'], powerSL)
    plt.plot(s92_V, s92_HP, marker='o', markersize=3, linestyle='')
    plt.plot(s92.vconfig['Power Curve']['Speeds'], s92.vconfig['Power Curve']['parasite'])
    plt.plot(s92.vconfig['Power Curve']['Speeds'], s92.vconfig['Power Curve']['induced'])
    plt.plot(s92.vconfig['Power Curve']['Speeds'], s92.vconfig['Power Curve']['profile'])
    plt.axis([0, 180, 0, 5000])
    #plt.legend(('Predicted', 'Published', 'parasite', 'induced', 'profile'), fontsize=labelfontsize)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xlabel('Airspeed (kts)', fontsize=labelfontsize)
    plt.ylabel('HP', fontsize=labelfontsize)
    plt.title('S-92', fontsize=titlefontsize)
    plt.tight_layout()
    plt.grid(True)

    plt.subplot(212)
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
    #xh59.write()
    plt.plot(xh59.vconfig['Power Curve']['Speeds'], xh59.vconfig['Power Curve']['PowersSL'])
    xh59_V = np.array([0, 21.966997, 42.561056, 63.471947, 85.122112, 87.762376, 89.663366, 91.669967, 105.927393, 108.567657, 120.712871, 125.887789, 126.627063, 130.112211, 136.765677, 136.554455, 143.841584, 146.481848, 151.339934, 154.719472])
    xh59_HP = np.array([1414.285714, 1168.163265, 925.714286, 813.673469, 732.857143, 800.816327, 854.081633, 786.122449, 879.795918, 817.346939, 912.857143, 1129.591837, 1050.612245, 1002.857143, 1054.285714, 1206.734694, 1307.755102, 1434.489796, 1520.816327, 1647.55102])
    plt.plot(xh59_V, xh59_HP, marker='o', markersize=3, linestyle='')
    plt.plot(xh59.vconfig['Power Curve']['Speeds'], xh59.vconfig['Power Curve']['parasite'])
    plt.plot(xh59.vconfig['Power Curve']['Speeds'], xh59.vconfig['Power Curve']['induced'])
    plt.plot(xh59.vconfig['Power Curve']['Speeds'], xh59.vconfig['Power Curve']['profile'])
    plt.axis([0, 180, 0, 2000])
    #plt.legend(('Predicted', 'Published'), fontsize=labelfontsize)
    plt.tick_params(labelsize=axislabelfontsize)
    plt.xlabel('Airspeed (kts)', fontsize=labelfontsize)
    plt.ylabel('HP', fontsize=labelfontsize)
    plt.title('XH-59', fontsize=titlefontsize)
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