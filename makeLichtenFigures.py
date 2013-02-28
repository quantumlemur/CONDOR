import math
import numpy as np
import scipy.ndimage
import matplotlib.pylab as pylab
import matplotlib.pyplot as plt


singlefigW = 3.4
singlefigH = 2.5
figDPI = 600
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




displayPlots = False
saveData = True
saveFigures = True
annotatePlots = True

plotScalingPlots = False
plotRfPlot = True

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

    if displayPlots: plt.show()
    if saveFigures: pylab.savefig('Output/Figures/RfPlot.png', dpi=figDPI, bbox_inches=0)


def ScalingPlots():
    # flat plate drag area
    plt.figure(num=None, figsize=(figW/1.2, figH/1.2), dpi=figDPI, facecolor='w', edgecolor='k')
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
    if saveFigures: pylab.savefig('Output/Figures/flatPlateDragScaling.png', bbox_inches=0)
    if displayPlots: plt.show()
    
    # engine weight
    plt.figure(num=None, figsize=(figW, figH), dpi=figDPI, facecolor='w', edgecolor='k')
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
    if saveFigures: pylab.savefig('Output/Figures/WeightsAndSFCScaling.png', bbox_inches=0)


if __name__ == '__main__':
    if plotScalingPlots: ScalingPlots()
    if plotRfPlot: RfPlot()