


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
from scipy import interpolate

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
    
    def __init__(self, c81File, rootChord, skip_header=0, skip_footer=0, taperRatio=1, tipTwist=0, rootCutout=0, segments=15):
        airfoildata = np.genfromtxt(c81File, skip_header=skip_header, skip_footer=skip_footer) # read in the airfoil file
        # generate the airfoil data splines.  They are assumed to be in degrees, and are converted to radians
        self.clspl = interpolate.splrep(airfoildata[:,0]*math.pi/180, airfoildata[:,1])
        self.cdspl = interpolate.splrep(airfoildata[:,0]*math.pi/180, airfoildata[:,2])
        self.cmspl = interpolate.splrep(airfoildata[:,0]*math.pi/180, airfoildata[:,3])
        # generate the piecewise data on the blade
        endpoints, self.dr = np.linspace(rootCutout, 1, segments+1, retstep=True)
        self.r = np.zeros(endpoints.size - 1)
        # set the r values as the midpoints of each segment
        for i in range(self.r.size):
            self.r[i] = (endpoints[i] + endpoints[i+1]) / 2
        # create the splines and store the twist and chord distribution
        twistspl = interpolate.splrep(np.array([rootCutout,1]), np.array([0,tipTwist*math.pi/180.]), k=1)  # k=1 for linear interpolation
        self.twist = interpolate.splev(self.r, twistspl)
        twist75 = interpolate.splev(.75, twistspl)
        self.twist -= twist75
        self.theta_tw = tipTwist * math.pi/180.
        chordspl = interpolate.splrep(np.array([rootCutout,1]), np.array([rootChord,rootChord*taperRatio]), k=1)  # k=1 for linear interpolation
        self.chord = interpolate.splev(self.r, chordspl)

    def cl(self, alpha):
        shape = alpha.shape
        alpha = np.reshape(alpha, alpha.size)
        alpha = interpolate.splev(alpha, self.clspl)
        alpha = np.reshape(alpha, shape)
        return alpha
    
    def cd(self, alpha):
        shape = alpha.shape
        alpha = np.reshape(alpha, alpha.size)
        alpha = interpolate.splev(alpha, self.cdspl)
        alpha = np.reshape(alpha, shape)
        return alpha
    
    def cm(self, alpha):
        shape = alpha.shape
        alpha = np.reshape(alpha, alpha.size)
        alpha = interpolate.splev(alpha, self.cmspl)
        alpha = np.reshape(alpha, shape)
        return alpha
    
    def a(self, alpha):
        shape = alpha.shape
        alpha = np.reshape(alpha, alpha.size)
        alpha = interpolate.splev(alpha, self.clspl, der=1)
        alpha = np.reshape(alpha, shape)
        return alpha


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

    def reinitializeTrimVars(self):
        # We'll just use .1 rad = 5.73 deg to initialize for now.  It'd be better to use closed-form estimates, but we'll add that later.
        self.beta_0 = .1 # precone
        self.theta_0 = .2
        self.theta_1c = 0.
        self.theta_1s = 0.

    def calcInflow(self, Fx, Fz, rho, V):
        # inflow calculations
        alpha_TPP = math.tan(Fx/Fz)
        thrust = math.sqrt(Fx**2 + Fz**2)
        diskArea = math.pi * self.R**2
        inflow = math.sqrt(thrust / (2.*rho*diskArea))
        change = 999.
        while change>.01:
            newinflow = thrust / (2.*rho*diskArea*math.sqrt((V*math.cos(alpha_TPP))**2 + (V*math.sin(alpha_TPP) + inflow)**2))
            change = abs(newinflow - inflow) / inflow
            inflow = newinflow
        #inflow_uniform_TPP = inflow #+ V*math.sin(alpha_TPP)
        #lambda_uniform_TPP = inflow_uniform_TPP / self.Vtip
        #mu = V / self.Vtip
        #lambda_i = inflow / self.Vtip
        #l = lambda_i + mu*math.sin(alpha_TPP)
        #lambda_nonuniform = lambda_i * (1 + 4./3.*mu/l)/(1.2+mu/l)*self.r/self.R*self.cospsi
        #if debug: pvar(locals(), ('mu', 'alpha_TPP', 'inflow', 'inflow_uniform_TPP', 'lambda_uniform_TPP'))
        inflow = inflow * (1. + (4./3.*V/inflow)/(1.2+V/inflow)*self.r/self.R*self.cospsi) # Glauert non-uniform inflow correction
        # import matplotlib.pyplot as plt
        # f = plt.figure()
        # f.add_subplot(111, projection='polar')
        # c = plt.contourf(self.psi, self.r, inflow)
        # plt.colorbar(c)
        # plt.title('inflow')
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
        alpha_TPP = math.tan(Fx/Fz) # tip path plane angle

        if debug: pvar(locals(), ('V', 'Fx', 'Fz'))

        dtheta_0_multiplier = math.sqrt(Fz**2 + Fx**2) / 10000000
        dtheta_0_multiplier = min(dtheta_0_multiplier, 0.1)
        dtheta_0_multiplier = max(dtheta_0_multiplier, 0.0001)

        # First we'll check if the pre-existing trim solution is reasonable. If not, re-initialize them.
        if not((self.beta_0>0) and (self.beta_0<math.pi/4) and (abs(self.theta_1c)<math.pi/4) and (abs(self.theta_1s)<math.pi/4)):
            self.reinitializeTrimVars()
        inflow = self.calcInflow(Fx=Fx, Fz=Fz, rho=rho, V=V)
        beta_0 = self.beta_0
        theta_0 = self.theta_0
        theta_1c = self.theta_1c
        theta_1s = self.theta_1s
        roll = 999
        pitch = 999
        L = 0
        P = 0
        loops = 0
        tol = tolerancePct / 100
        while not (abs(Fz-L)/Fz<tol and abs(lastP-P)/P<tol) and loops<maxSteps and np.isfinite(P) and abs(theta_0)<math.pi:
            loops += 1
            #### These following equations are from slides ~70-80 in Part2.ppt of AE6070 notes.  All angles relative to flight path?
            # find the effective blade section angle of attack
            beta = beta_0
            U_T = omega*r + V*sinpsi # local tangential velocity.
            U_P = inflow + V*beta*cospsi # local perpendicular velocity.
            # A = theta_1c-beta_1s
            # B = theta_1s+beta_1c
            theta = theta_0 + blade.theta_tw*r/R + theta_1c*cospsi - theta_1s*sinpsi
            phi = np.arctan(U_P/U_T)
            alphaEffective = theta - phi
            #alphaEffective = 1/U_T * (omega*r*(theta_0 + blade.theta_tw*r/R + (theta_1c)*cospsi + (theta_1s)*sinpsi)*V*(theta_0 + blade.theta_tw*r/R)*sinpsi + V*(theta_1c)*cospsi*sinpsi + V*(theta_1s)*sinpsi**2 - V*beta_0*cospsi - V*alpha_TPP - inflow)
            #### alphaEffective = thetaFixed - np.arctan((R*inflow*omega + r*omega*(beta_1s*cospsi-beta_1c*sinpsi) + V*math.cos(alpha_TPP-beta_1c)*cospsi*(beta_0 + beta_1c*cospsi + beta_1s*sinpsi)) / (r*omega + V*math.cos(alpha_TPP-beta_1c)*sinpsi)) + theta_1c*cospsi + theta_1s*sinpsi # This is the full equation for the line above.  Retained only for reference and troubleshooting.
            # get the cl, cd, and cm data for blade sections.  Since we're using lookup tables, this is a quasi-steady solution.  We may want to add in corrections for unstead aerodynamic effects and dynamic stall in the future.  Dynamic stall is also important to find the vibration levels of the vehicle in high speed forward flight.
            alphaEffective[abs(alphaEffective)>math.pi] = math.pi
            cl = self.blade.cl(alphaEffective)
            cd = self.blade.cd(alphaEffective)
            #cm = self.blade.cm(alphaEffective)
            # apply Prandtl-Glauert compressibility correction
            U_total = np.sqrt(U_T**2 + U_P**2) # Although we do not, we can assume U_T>>U_P, which is only untrue if U_T is small (i.e. on the retreating side in high-speed flight), in which case mach numbers will be small anyway, so should be ok.
            mach = U_total / speedOfSound
            cl = cl / np.sqrt(1-mach**2)
            # Find the rotor sectional (1D) lift and drag
            dL = .5 * rho * U_total**2 * chord * cl
            dD = .5 * rho * U_total**2 * chord * cd
            #dL[~np.isfinite(dL)] = 0
            #dD[~np.isfinite(dD)] = 0
            # Calculate the piecewise (2D) lift and drag
            cosphi = np.cos(phi)
            sinphi = np.sin(phi)
            dT = (dL*cosphi - dD*sinphi) * dr * ( dpsi)
            dDinduced = dL*sinphi * dr * dpsi
            dDprofile = dD*cosphi * dr * dpsi
            # Integrate over the rotor surface
            T = np.sum(dT) * numblades
            Pinduced = np.sum(dDinduced * U_T) * numblades / 550
            Pprofile = np.sum(dDprofile * U_T) * numblades / 550
            lastP = P
            P = Pinduced + Pprofile
            # Calculate trim angles
            #beta_0 = lockNumber * (theta_0/8*(1+mu**2) - mu**2/60*blade.theta_tw - lambda_uniform_TPP/6 + mu*theta_1s/6)
            #theta_1s = -8./3.*mu*(theta_0 - 3./4.*lambda_uniform_TPP) / (1 + 3./2.*mu**2)
            #theta_1c = (-4./3.*mu*beta_0) / (1. + .5*mu**2)
            beta_0 = lockNumber / (2.*math.pi) * T / (.5*rho*math.pi*R**2*Vtip**2) # 
            pitch = np.sum(dT*cospsi) / T / 1000
            roll = np.sum(dT*sinpsi) / T / 1000
            theta_1c -= pitch
            theta_1s += roll
            # Find vertical lift and adjust collective
            L = T * math.cos(alpha_TPP)
            liftDeficitPct = (Fz - L) / Fz
            dtheta_0 = liftDeficitPct * dtheta_0_multiplier
            # cap the max change at 0.1
            if abs(dtheta_0) > 0.1:
                dtheta_0 /= abs(dtheta_0 * 10)
            theta_0 += dtheta_0            
            if debug: 
                coll = theta_0 * 180/math.pi
                b0 = beta_0 * 180/math.pi
                t1s = theta_1s * 180/math.pi
                t1c = theta_1c * 180/math.pi
                pvar(locals(), ('Fz', 'L', 'T', 'coll', 'b0', 't1c', 't1s', 'Pinduced', 'Pprofile', 'P'))
        self.inflow = inflow
        self.theta_0 = theta_0
        self.theta_1c = theta_1c
        self.theta_1s = theta_1s
        self.beta_0 = beta_0

        if abs(Fz-L)/Fz<tol and abs(lastP-P)/P<tol:
            P_total = Pinduced + Pprofile + Fx*V/550
        else:
            P_total = np.nan

        # import matplotlib.pyplot as plt
        # f = plt.figure()
        # f.add_subplot(331, projection='polar')
        # c = plt.contourf(psi, r, dT)
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
    from time import clock
    startTime = clock()
    rho = 0.001207 # slugs per ft^3
    f = 15. # square feet
    Vtip = 650. # ft/s
    R = 30. # feet
    omega = Vtip / R # rad/s
    b = Blade(c81File='Config/S809_Cln.dat', skip_header=12, skip_footer=1, rootChord=1./15, taperRatio=.8, tipTwist=-8., rootCutout=.2, segments=100)
    r = Rotor(b, psiSegments=100, Vtip=Vtip, radius=R, numblades=4)
    # speeds = np.arange(0, 5, .5)
    # Pi = np.zeros(speeds.size)
    # Ppr = np.zeros(speeds.size)
    # Ppa = np.zeros(speeds.size)
    # for i in xrange(speeds.size):
    #     V = speeds[i]
    #     print V
    #     Fhorizontal = 1./2 * rho * V**2 * f
    #     Fvertical = 15000. # pounds
    #     Pi[i], Ppr[i] = r.trim(tolerancePct=.1, V=V, rho=rho, speedOfSound=1026., Fx=Fhorizontal, Fz=Fvertical, maxSteps=10000)
    #     Ppa[i] =  Fhorizontal*V/550
    # import matplotlib.pyplot as plt
    # plt.figure()
    # plt.plot(speeds/1.687, Pi+Ppr+Ppa)
    # plt.plot(speeds/1.687, Pi)
    # plt.plot(speeds/1.687, Ppr)
    # plt.plot(speeds/1.687, Ppa)
    # plt.legend(('total', 'induced', 'profile', 'parasite'))
    # plt.xlabel('speed (kts)')
    # plt.ylabel('power (hp)')
    # plt.show()
    V = 162.5
    mu = V/Vtip
    Fhorizontal = .5 * rho * V**2 * f
    Fvertical = 45000. # pounds
    print r.trim(tolerancePct=1., V=V, rho=rho, speedOfSound=1026., Fx=Fhorizontal, Fz=Fvertical, maxSteps=1000)
    stopTime = clock()
    elapsed = stopTime - startTime
    if debug: print('Elapsed time: %f' % elapsed)