


# These are just a few notes that came out of a meeting, in which we touched on rotor design.  I'm keeping them here for future reference,
# but they apply more to my own rotor project than to the code as a whole.  Or maybe for some default blade/rotor or something.


#NASA Report on Advanced Tech, page 37
#-9 degrees twist
#
#VR-12 from 0 to .85,
#linear transition from .85 to .95
#VR-15 from .95 to tip
#no tip sweep
#
#Leo Dedone's patent for airfoild

# target 60-80 lb/sq ft wing loading, no more than 100
# nominal wing incidence at 3deg
# can use propeller efficiency plot from p.55 om NASA report
# if you find BEM equations for propellers that include J as the advance ratio, be careful because it has some different definition







import math
import numpy as np
import random
#from scipy import interpolate

debug = False

def pvar(locals_, vars_):
    s = ['%s: %.3f' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)

def pdeg(locals_, vars_):
    s = ['%s: %.2f' % (var*180./math.pi, locals_[var]) for var in vars_]
    print '     '.join(s)


class Blade:
    # This is a very simple class intended to just hold the configuration for a single blade.
    # Broken down into radial pieces, and with the c81 file read in and interpolation splines
    # generated so that cl, cd, etc. can be returned easily later.  It will contain a
    # non-dimensional blade, with everything referenced to a radius of 1.
    
    def __init__(self, airfoildata, averageChord, skip_header=0, skip_footer=0, taperRatio=1, tipTwist=0, rootCutout=0, segments=15, dragDivergenceMachNumber=.85):
        self.dragDivergenceMachNumber = dragDivergenceMachNumber
        #airfoildata = np.genfromtxt(c81File, skip_header=skip_header, skip_footer=skip_footer) # read in the airfoil file
        # generate the airfoil data splines.  They are assumed to be in degrees, and are converted to radians

        # self.clspl = interpolate.splrep(airfoildata[:,0]*math.pi/180, airfoildata[:,1])
        # self.cdspl = interpolate.splrep(airfoildata[:,0]*math.pi/180, airfoildata[:,2])
        # self.cmspl = interpolate.splrep(airfoildata[:,0]*math.pi/180, airfoildata[:,3])
        self.alphadata = airfoildata[:,0]*math.pi/180
        self.cldata = airfoildata[:,1]
        self.cddata = airfoildata[:,2]
        self.cmdata = airfoildata[:,3]
        # generate the piecewise data on the blade
        endpoints, self.dr = np.linspace(rootCutout, 1, segments+1, retstep=True)
        self.r = np.zeros(endpoints.size - 1)
        # set the r values as the midpoints of each segment
        for i in range(self.r.size):
            self.r[i] = (endpoints[i] + endpoints[i+1]) / 2
        # create the splines and store the twist and chord distribution
        #twistspl = interpolate.splrep(np.array([rootCutout,1]), np.array([0,tipTwist*math.pi/180.]), k=1)  # k=1 for linear interpolation
        self.twist = np.interp(self.r, [rootCutout,1], [0,tipTwist*math.pi/180.]) #interpolate.splev(self.r, twistspl)
        twist75 = np.interp(.75, [rootCutout,1], [0,tipTwist*math.pi/180.]) #interpolate.splev(.75, twistspl)
        self.twist -= twist75
        self.theta_tw = tipTwist * math.pi/180.
        rootChord = 2 * averageChord / (taperRatio + 1)
        #chordspl = interpolate.splrep(np.array([rootCutout,1]), np.array([rootChord,rootChord*taperRatio]), k=1)  # k=1 for linear interpolation
        self.chord = np.interp(self.r, [rootCutout,1], [rootChord,rootChord*taperRatio]) #interpolate.splev(self.r, chordspl)

    def cl(self, alpha):
        shape = alpha.shape
        alpha = np.reshape(alpha, alpha.size)
        #alpha = interpolate.splev(alpha, self.clspl)
        cls = np.interp(alpha, self.alphadata, self.cldata)
        cls = np.reshape(cls, shape)
        return cls
    
    def cd(self, alpha):
        shape = alpha.shape
        alpha = np.reshape(alpha, alpha.size)
        #alpha = interpolate.splev(alpha, self.cdspl)
        cds = np.interp(alpha, self.alphadata, self.cddata)
        cds = np.reshape(cds, shape)
        return cds
    
    def cm(self, alpha):
        shape = alpha.shape
        alpha = np.reshape(alpha, alpha.size)
        #alpha = interpolate.splev(alpha, self.cmspl)
        cms = np.interp(alpha, self.alphadata, self.cmdata)
        cms = np.reshape(cms, shape)
        return cms

    def write(self):
        # right now this function just writes out some of the blade data to test the reading and interpolation routines.  Should remove it once the class is finalized.
        np.savetxt('Config/Blade.csv', np.transpose((self.r, self.theta)))
        alphas = np.arange(15)
        cl = self.cl(alphas)
        cd = self.cd(alphas)
        cm = self.cm(alphas)
        np.savetxt('Config/Airfoil.csv', np.transpose((alphas, cl, cd, cm)))


class Rotor:
    """This class holds the rotor and actually deals with trimming it and whatnot.

    It contains dimensional quantities."""
    
    def __init__(self, blade, psiSegments, Vtip, radius, numblades):
        self.blade = blade
        self.Vtip = Vtip
        self.radius = radius
        self.R = radius
        self.numblades = numblades
        self.omega = Vtip / radius
        # set up 1D arrays of psi and r...
        self.psi1D, self.dpsi = np.linspace(0, 2*math.pi, psiSegments, endpoint=False, retstep=True)
        r1D = self.blade.r
        # ...and convert them to 2D, plus chord and theta
        self.r, self.psi = np.meshgrid(r1D, self.psi1D)
        self.chord, self.psi = np.meshgrid(blade.chord, self.psi1D)
        # dimensionalize some of the blade data
        self.chord = self.chord * radius
        self.r = self.r * radius
        self.dr = self.blade.dr * radius
        # calculate the solidity
        self.bladeArea = np.sum(self.chord) * self.dr * self.dpsi * numblades / (2*math.pi)
        self.diskArea = math.pi * radius**2
        self.solidity = self.bladeArea / self.diskArea
        # precompute and store the values of sin(psi) and cos(psi) since they're static and we're going to use them a lot later on
        self.cospsi = np.cos(self.psi)
        self.sinpsi = np.sin(self.psi)
        # initialize the trim variables such that they'll get set when we first try to trim
        self.beta_0 = 9999999.
        self.beta_1c = 9999999.
        self.beta_1s = 9999999.
        self.theta_0 = 9999999.
        self.theta_1c = 9999999.
        self.theta_1s = 9999999.
        self.inflow = 9999999.
        self.power = np.nan

    def reinitializeTrimVars(self):
        # We'll just use .1 rad = 5.73 deg to initialize for now.  It'd be better to use closed-form estimates, but we'll add that later.
        self.beta_0 = .1 # precone
        self.theta_0 = .2
        self.theta_1c = 0.
        self.theta_1s = 0.

    def calcInflow(self, Fx, Fz, rho, V, maxSteps):
        # inflow calculations
        alpha_TPP = math.tan(Fx/Fz)
        thrust = math.sqrt(Fx**2 + Fz**2)
        diskArea = math.pi * self.R**2
        inflow = math.sqrt(thrust / (2.*rho*diskArea))
        change = 999.
        steps = 0
        # I believe these inflow equations are only valid for small alpha_TPP, so are not valid for propellers
        while change>.01 and steps<maxSteps:
            # Tip loss modelby Prandlt (documented in Leishman)
            f = self.numblades/2. * (1-self.r/self.R)/(inflow/self.Vtip)
            F = 2./math.pi * np.arccos(np.exp(-f))
            #F[~np.isfinite(F)] = 0.001
            newinflow = thrust / (2.*rho*diskArea*np.sqrt((V*math.cos(alpha_TPP))**2 + (V*math.sin(alpha_TPP) + F*inflow)**2))
            change = abs(np.sum(newinflow) - np.sum(inflow)) / np.sum(inflow)
            inflow = newinflow
            steps += 1
        uniform = (inflow * self.r) / self.r
        inflow = inflow * (1. + (4./3.*V/inflow)/(1.2+V/inflow)*self.r/self.R*self.cospsi) # Glauert non-uniform inflow correction
        # import matplotlib.pyplot as plt
        # fig = plt.figure()
        # fig.add_subplot(221, projection='polar')
        # c = plt.contourf(self.psi, self.r, f)
        # plt.colorbar(c)
        # plt.title('f')
        # fig.add_subplot(222, projection='polar')
        # c = plt.contourf(self.psi, self.r, F)
        # plt.colorbar(c)
        # plt.title('F')
        # fig.add_subplot(223, projection='polar')
        # c = plt.contourf(self.psi, self.r, inflow)
        # plt.colorbar(c)
        # plt.title('inflow')
        # fig.add_subplot(224, projection='polar')
        # c = plt.contourf(self.psi, self.r, uniform)
        # plt.colorbar(c)
        # plt.title('uniform')
        # plt.show()
        return inflow
    
    def trim(self, tolerancePct, V, speedOfSound, rho, Fx, Fz, maxSteps):
        """Attempts to trim the rotor at given conditions.  Will re-use trim variables from last time if possible.
        Note that "drag" and "weight" in the variables are really just the veritcal and horizontal components of
        the force that the rotor is generating."""
        psi = self.psi
        r = self.r
        R = self.R
        chord = self.chord
        omega = self.omega
        dr = self.dr
        dpsi = self.dpsi
        numblades = self.numblades
        cospsi = self.cospsi
        sinpsi = self.sinpsi
        Vtip = self.Vtip
        blade = self.blade
        mu = V / Vtip
        lockNumber = 8. # assumed estimate
        alpha_TPP = math.atan(Fx/Fz) # tip path plane angle
        if debug: print alpha_TPP*180./math.pi
        totalThrust = math.sqrt(Fx**2 + Fz**2)

        if debug: pvar(locals(), ('V', 'Fx', 'Fz'))

        dtheta_0_multiplier = math.sqrt(Fz**2 + Fx**2) / 1000000
        dtheta_0_multiplier = min(dtheta_0_multiplier, 0.1)
        dtheta_0_multiplier = max(dtheta_0_multiplier, 0.0001)

        # First we'll check if the pre-existing trim solution is reasonable. If not, re-initialize them.
        if (self.beta_0<0) or (self.beta_0>math.pi/6) or (abs(self.theta_1c)>math.pi/6) or (abs(self.theta_1s)>math.pi/6) or math.isnan(self.power):
            self.reinitializeTrimVars()
        inflow = self.calcInflow(Fx=Fx, Fz=Fz, rho=rho, V=V, maxSteps=maxSteps)
        beta_0 = self.beta_0
        theta_0 = self.theta_0
        theta_1c = self.theta_1c
        theta_1s = self.theta_1s
        roll = 999
        pitch = 999
        L = 0
        P = 1
        T = 0
        lastP = 0
        steps = 0
        CChanges = []
        TChanges = []
        tol = tolerancePct / 100
        liftDeficitPct = 9
        while np.isfinite(P) and steps<maxSteps and abs(theta_0)<math.pi/6 and not (abs(liftDeficitPct)<tol and abs(lastP-P)/P<tol) and abs(P)<40000:
            steps += 1
            # find the effective blade section angle of attack
            beta = beta_0
            U_T = omega*r + V*sinpsi*math.cos(alpha_TPP) # local tangential velocity.
            U_P = inflow + V*beta*cospsi*math.sin(alpha_TPP) # local perpendicular velocity.
            theta = theta_0 + blade.theta_tw*r/R + theta_1c*cospsi - theta_1s*sinpsi
            phi = np.arctan(U_P/U_T)
            alphaEffective = theta - phi
            # get the cl and cd data for blade sections.  Since we're using lookup tables, this is a quasi-steady solution.  We may want to add in corrections for unstead aerodynamic effects and dynamic stall in the future.  Dynamic stall is also important to find the vibration levels of the vehicle in high speed forward flight.
            alphaEffective[abs(alphaEffective)>math.pi] = math.pi
            cl = self.blade.cl(alphaEffective)
            cd = self.blade.cd(alphaEffective)
            # apply Prandtl-Glauert compressibility correction
            U_total = np.sqrt(U_T**2 + U_P**2) # Although we do not, we can assume U_T>>U_P, which is only untrue if U_T is small (i.e. on the retreating side in high-speed flight), in which case mach numbers will be small anyway, so should be ok.
            mach = U_total / speedOfSound
            cd[mach>blade.dragDivergenceMachNumber] = (mach[mach>blade.dragDivergenceMachNumber]-blade.dragDivergenceMachNumber)*.6 + cd[mach>blade.dragDivergenceMachNumber] # drag divergence
            mach[mach>=1] = .999999
            cl = cl / np.sqrt(1-mach**2)
            #cl[~np.isfinite(cl)] = 0. # if local mach>1, set cl=0.  This correction doesn't apply in those ranges, but this is probably acceptable and will allow trim solutions even with local M>1.  In combination with drag divergence, it should be an OK approximation.
            # Find the rotor sectional (1D) lift and drag
            dL = .5 * rho * U_total**2 * chord * cl
            dD = .5 * rho * U_total**2 * chord * cd
            # Calculate the piecewise (2D) lift and drag
            cosphi = np.cos(phi)
            sinphi = np.sin(phi)
            dT = (dL*cosphi - dD*sinphi) * dr * ( dpsi) / (2*math.pi)
            dDinduced = dL*sinphi * dr * dpsi / (2*math.pi)
            dDprofile = dD*cosphi * dr * dpsi / (2*math.pi)
            # Integrate over the rotor surface
            lastT = T
            T = np.sum(dT) * numblades
            Pinduced = np.sum(dDinduced * U_T) * numblades / 550
            Pprofile = np.sum(dDprofile * U_T) * numblades / 550
            lastP = P
            P = Pinduced + Pprofile
            # Calculate trim angles
            beta_0 = lockNumber / (2.*math.pi) * T / (.5*rho*math.pi*R**2*Vtip**2) # should replace this with a real calculation involving blade mass, but it's good enough for now
            pitch = np.sum(dT*cospsi) / T / 1000
            roll = np.sum(dT*sinpsi) / T / 1000
            theta_1c -= pitch
            theta_1s += roll
            # Find vertical lift and adjust collective
            L = T * math.cos(alpha_TPP)
            liftDeficitPct = (totalThrust - T) / abs(T) #abs(random.choice([totalThrust, T]))
            dtheta_0 = liftDeficitPct * dtheta_0_multiplier
            # cap the max change at 0.1
            if abs(dtheta_0) > 0.005:
                dtheta_0 /= abs(dtheta_0 / 0.005)
                #dtheta_0 += random.uniform(-0.002, 0.002)
            # and minimum at 0.001
            #if abs(dtheta_0) < 0.0000001:
            #    dtheta_0 /= abs(dtheta_0 / 0.0000001)
            CChanges.append(dtheta_0 > 0) # true if theta is going up
            TChanges.append(T-lastT > 0) # true if thrust is going up
            if len(CChanges) > 200:
                if (all(CChanges) and not any(TChanges)) or (not any(CChanges) and all(TChanges)): # collective has been going up always and thrust has never been going up, or vice versa
                    if debug: print('Consistently wrong direction of change!')
                    P = float('nan') # this is the easiest way to break the loop...
                CChanges.pop(0)
                TChanges.pop(0)
            theta_0 += dtheta_0            
            if debug: 
                coll = theta_0 * 180/math.pi
                b0 = beta_0 * 180/math.pi
                t1s = theta_1s * 180/math.pi
                t1c = theta_1c * 180/math.pi
                total = math.sqrt(Fx**2 + Fz**2)
                dthet = dtheta_0 #* 1000
                pvar(locals(), ('V', 'steps', 'total', 'T', 'dthet', 'coll', 'b0', 't1c', 't1s', 'Pinduced', 'Pprofile', 'P'))
        self.inflow = inflow
        self.theta_0 = theta_0
        self.theta_1c = theta_1c
        self.theta_1s = theta_1s
        self.beta_0 = beta_0

        if abs(liftDeficitPct)<tol and abs(lastP-P)/lastP<tol and abs(theta_0)<math.pi/6 and abs(P)<40000:
            P_total = Pinduced + Pprofile
        else:
            if debug: print('%s < %s       %s < %s' % (abs(Fz-L)/Fz, tol, tol, tol))
            P_total = np.nan
        if debug: print P_total
        if debug: print alpha_TPP * 180/math.pi

        # import matplotlib.pyplot as plt
        # f = plt.figure()
        # f.add_subplot(331, projection='polar')
        # c = plt.contourf(psi, r, dT, np.arange(-.25, 2, 0.01))
        # plt.colorbar(c)
        # plt.title('dT')

        # f.add_subplot(332, projection='polar')
        # c = plt.contourf(psi, r, alphaEffective*180/math.pi, np.arange(-20, 20))
        # plt.colorbar(c)
        # plt.title('alpha')

        # f.add_subplot(333, projection='polar')
        # c = plt.contourf(psi, r, U_total)
        # plt.colorbar(c)
        # plt.title('U_total')

        # f.add_subplot(334, projection='polar')
        # c = plt.contourf(psi, r, cl)
        # plt.colorbar(c)
        # plt.title('cl')

        # f.add_subplot(335, projection='polar')
        # c = plt.contourf(psi, r, cd)
        # plt.colorbar(c)
        # plt.title('cd')

        # f.add_subplot(336, projection='polar')
        # c = plt.contourf(psi, r, theta*180/math.pi)
        # plt.colorbar(c)
        # plt.title('theta')

        # f.add_subplot(337, projection='polar')
        # c = plt.contourf(psi, r, U_P)
        # plt.colorbar(c)
        # plt.title('U_P')

        # f.add_subplot(338, projection='polar')
        # c = plt.contourf(psi, r, U_T)
        # plt.colorbar(c)
        # plt.title('U_T')

        # f.add_subplot(339, projection='polar')
        # c = plt.contourf(psi, r, phi*180/math.pi, np.arange(-20, 20))
        # plt.colorbar(c)
        # plt.title('phi')

        # plt.show()

        self.power = P_total
        return P_total #(Pinduced, Pprofile)




    

            # dF_x = - dL*sinphi*cospsi - dD*cosphi*cospsi
            # dF_y = - dL*sinphi*sinpsi - dD*cosphi*sinpsi
            # dF_z = - dL*cosphi + dD*sinphi
            # dQ = - dL*sinphi - dD*cosphi
            # # Integrate over the disk surface
            # dF_x[~np.isfinite(dF_x)] = 0
            # dF_y[~np.isfinite(dF_y)] = 0
            # dF_z[~np.isfinite(dF_z)] = 0
            # dQ[~np.isfinite(dQ)] = 0
            # F_x = np.sum(dF_x * dr * dpsi) * numblades
            # F_y = np.sum(dF_y * dr * dpsi) * numblades
            # F_z = np.sum(dF_z * dr * dpsi) * numblades
            # Q = np.sum(dQ * dr * r * dpsi) * numblades

    def plot(self):
        pass

    
if __name__ == '__main__':
    debug = True
    from time import clock
    from configobj import ConfigObj
    from validate import Validator
    startTime = clock()
    GW = 60000.
    v = ConfigObj('Config/vehicle.cfg', configspec='Config/vehicle.configspec')
    m = ConfigObj('Config/mission.cfg', configspec='Config/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    rho = 0.0024
    f = 0.25 * GW**.5 * (1-v['Body']['DragTechImprovementFactor'])
    print f
    Vtip = v['Main Rotor']['TipSpeed'] # ft/s
    R = math.sqrt(GW / (math.pi * v['Main Rotor']['DiskLoading'] * v['Main Rotor']['NumRotors']))
    v['Main Rotor']['Radius'] = R
    v['Main Rotor']['DiskArea'] = math.pi * v['Main Rotor']['Radius']**2 * v['Main Rotor']['NumRotors']
    v['Main Rotor']['AverageChord'] = v['Main Rotor']['DiskArea']*v['Main Rotor']['Solidity'] / (v['Main Rotor']['Radius']*(1-v['Main Rotor']['RootCutout'])*v['Main Rotor']['NumBlades'])
    omega = Vtip / R # rad/s
    c81File='Config/%s'%v['Main Rotor']['AirfoilFile']
    airfoildata = np.genfromtxt(c81File, skip_header=0, skip_footer=0) # read in the airfoil file
    blade = Blade(airfoildata=airfoildata, skip_header=0, skip_footer=0, averageChord=v['Main Rotor']['AverageChord']/v['Main Rotor']['Radius'], taperRatio=v['Main Rotor']['TaperRatio'], tipTwist=v['Main Rotor']['TipTwist'], rootCutout=v['Main Rotor']['RootCutout']/v['Main Rotor']['Radius'], segments=v['Simulation']['numBladeElementSegments'], dragDivergenceMachNumber=v['Main Rotor']['DragDivergenceMachNumber'])
    rotor = Rotor(blade, psiSegments=v['Simulation']['numBladeElementSegments'], Vtip=v['Main Rotor']['TipSpeed'], radius=v['Main Rotor']['Radius'], numblades=v['Main Rotor']['NumBlades'])
    bladeArea = np.sum(blade.chord * rotor.radius * blade.dr * rotor.radius * rotor.numblades)
    diskArea = math.pi * rotor.radius**2
    solidity = bladeArea / diskArea
    pvar(locals(), ('bladeArea', 'diskArea', 'solidity'))
    speeds = np.arange(300, 305, 5)
    Pi = np.zeros(speeds.size)
    Ppr = np.zeros(speeds.size)
    Ppa = np.zeros(speeds.size)
    P_tot = np.zeros(speeds.size)
    for i in xrange(speeds.size):
        V = speeds[i]
        print V
        Fhorizontal = 1./2 * rho * V**2 * f
        Fvertical = GW  # pounds
        P_tot[i] = rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], V=V, rho=rho, speedOfSound=1026., Fx=Fhorizontal, Fz=Fvertical, maxSteps=v['Simulation']['MaxSteps']) + Fhorizontal*V/550
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(speeds/1.687, P_tot)
    #plt.legend(('total', 'induced', 'profile', 'parasite'))
    plt.xlabel('speed (kts)')
    plt.ylabel('power (hp)')
    stopTime = clock()
    plt.show()
    # V = 800.
    # mu = V/Vtip
    # Fhorizontal = .5 * rho * V**2 * f
    # Fvertical = GW # pounds
    # power =  rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], V=V, rho=rho, speedOfSound=1026., Fx=Fhorizontal, Fz=Fvertical, maxSteps=v['Simulation']['MaxSteps'])
    elapsed = stopTime - startTime
    print('elapsed time: %d' % elapsed)
