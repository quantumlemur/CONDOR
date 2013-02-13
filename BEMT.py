


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

debug = True

def pvar(locals_, vars_):
    s = ['%s: %f' % (var, locals_[var]) for var in vars_]
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
        twistspl = interpolate.splrep(np.array([rootCutout,1]), np.array([0,tipTwist]), k=1)  # k=1 for linear interpolation
        self.twist = interpolate.splev(self.r, twistspl)
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

    It contains dinensional quantities."""
    
    def __init__(self, blade, psiSegments, omega, radius, pitch, numblades):
        self.blade = blade
        self.omega = omega
        self.radius = radius
        self.R = radius
        self.pitch = pitch
        self.numblades = numblades
        # set up 1D arrays of psi and r...
        self.psi1D, psistep = np.linspace(0, 2*math.pi, psiSegments, endpoint=False, retstep=True)
        r1D = self.blade.r
        # ...and convert them to 2D, plus chord and theta
        self.r, self.psi = np.meshgrid(r1D, self.psi1D)
        self.chord, self.psi = np.meshgrid(blade.chord, self.psi1D)
        # dimensionalize some of the blade data
        self.chord = self.blade.chord * radius
        self.r = self.blade.r * radius
        self.dr = self.blade.dr * radius
        # set the pitch values
        self.setPitch(pitch)
        # precompute and store the values of sin(psi) and cos(psi) since they're static and we're going to use them a lot later on
        self.cospsi = np.cos(self.psi)
        self.sinpsi = np.sin(self.psi)
        # initialize the trim variables such that they'll get set when we first try to trim
        self.beta_0 = 9999999.
        self.beta_1c = 9999999.
        self.beta_1s = 9999999.
        self.theta_1c = 9999999.
        self.theta_1s = 9999999.
        self.inflow = 9999999.
    
    def setPitch(self, pitch):
        self.pitch = pitch
        theta1D = pitch + self.blade.twist
        self.thetaFixed, psi = np.meshgrid(theta1D, self.psi1D)

    def reinitializeTrimVars(self, Fx, Fz, rho, V):
        # We'll just use .1 rad = 5.73 deg to initialize for now.  It'd be better to use closed-form estimates, but we'll add that later.
        self.beta_0 = .1 # precone
        self.beta_1c = .1
        self.beta_1s = .1
        self.theta_1c = .1
        self.theta_1s = .1
        # inflow calculations
        self.alpha_tpp = math.tan(Fx/Fz)
        thrust = math.sqrt(Fx**2 + Fz**2)
        diskArea = math.pi * self.R**2
        inflow = .1
        change = 999.
        while change>.01:
            newinflow = thrust / (2.*rho*diskArea*math.sqrt((V*math.cos(self.alpha_tpp))**2 + (V*math.sin(self.alpha_tpp) + inflow)**2))
            change = abs(newinflow - inflow) / inflow
            inflow = newinflow
        self.inflow = inflow
    
    def trim(self, tolerance, V, speedOfSound, rho, Fx, Fy, Fz):
        """Attempts to trim the rotor at given conditions.  Will re-use trim variables from last time if possible.
        Note that "drag" and "weight" in the variables are really just the veritcal and horizontal components of
        the force that the rotor is generating."""
        psi = self.psi
        r = self.r
        R = self.R
        chord = self.chord
        thetaFixed = self.thetaFixed
        omega = self.omega
        dr = self.dr
        numblades = self.numblades
        cospsi = self.cospsi
        sinpsi = self.sinpsi
        #alpha_TPP = math.tan(drag/weight) # tip path plane angle

        # First we'll check if the pre-existing trim solution is reasonable. If not, re-initialize them.
        if not((self.inflow<1000) and (self.beta_0>0) and (self.beta_0<math.pi/4) and (abs(self.beta_1c)<math.pi/4) and (abs(self.beta_1s)<math.pi/4) and (abs(self.theta_1c)<math.pi/4) and (abs(self.theta_1s)<math.pi/4)):
            self.reinitializeTrimVars(Fx=Fx, Fz=Fz, rho=rho, V=V)
        inflow = self.inflow
        beta_0 = self.beta_0
        beta_1c = self.beta_1c
        beta_1s = self.beta_1s
        theta_1c = self.theta_1c
        theta_1s = self.theta_1s
        F_x = 0
        F_y = 0
        F_z = 0
        if debug: print('TARGETS:     F_x=%d      F_y=%d      F_z=%d' % (Fx, Fy, Fz))
        while abs(F_x-Fx)>1 or abs(F_y-Fy)>1 or abs(F_z-Fz)>1:
            #### These following equations are from slides ~70-80 in Part2.ppt of AE6070 notes.  All angles relative to flight path?
            # find the effective blade section angle of attack
            theta = thetaFixed + theta_1c*cospsi + theta_1s*sinpsi # local flow angle
            betadot = omega * (beta_1s*cospsi - beta_1c*sinpsi) # local flapping rate
            beta = beta_0 + beta_1c*cospsi + beta_1s*sinpsi # local flapping angle
            U_T = omega*r + V*sinpsi # local tangential velocity.  This assumes small shaft angle alpha_s
            U_P = omega*R*inflow + r*betadot + V*beta*cospsi # local perpendicular velocity.  This also assumes small shaft angle alpha_s
            phi = np.arctan(U_P/U_T) # local flow angle
            alphaEffective = theta - phi # effective angle of attack that the blade section sees
            #### alphaEffective = thetaFixed - np.arctan((R*inflow*omega + r*omega*(beta_1s*cospsi-beta_1c*sinpsi) + V*math.cos(alpha_TPP-beta_1c)*cospsi*(beta_0 + beta_1c*cospsi + beta_1s*sinpsi)) / (r*omega + V*math.cos(alpha_TPP-beta_1c)*sinpsi)) + theta_1c*cospsi + theta_1s*sinpsi # This is the full equation for the line above.  Retained only for reference and troubleshooting.
            # get the cl, cd, and cm data for blade sections.  Since we're using lookup tables, this is a quasi-steady solution.  We may want to add in corrections for unstead aerodynamic effects and dynamic stall in the future.  Dynamic stall is also important to find the vibration levels of the vehicle in high speed forward flight.
            print alphaEffective.shape
            cl = self.blade.cl(alphaEffective)
            cd = self.blade.cd(alphaEffective)
            cm = self.blade.cm(alphaEffective)
            # apply Prandtl-Glauert compressibility correction
            U_total = np.sqrt(U_T**2 + U_P**2)
            mach = U_total / speedOfSound # Can this be simplified to U_T/speedOfSound ? Should check that; it'll be slightly faster if so.
            cl = cl / np.sqrt(1-mach)
            # Find the rotor sectional lift and drag
            dL = .5 * rho * chord * U_total**2 * cl
            dD = .5 * rho * chord * U_total**2 * cd
            # rotate the sectional lift and drag into x, y, and z forces, plus torque
            sinphi = np.sin(phi)
            cosphi = np.cos(phi)
            #### Do I need to rotate from the TPP to the flight-path-plane?  Not sure... we'll find out later
            dF_x = - dL*sinphi*cospsi - dD*cosphi*cospsi
            dF_y = - dL*sinphi*sinpsi - dD*cosphi*sinpsi
            dF_z = - dL*cosphi + dD*sinphi
            dQ = - dL*sinphi - dD*cosphi
            # Integrate over the disk surface
            dF_x[~np.isfinite(dF_x)] = 0
            dF_y[~np.isfinite(dF_y)] = 0
            dF_z[~np.isfinite(dF_z)] = 0
            dQ[~np.isfinite(dQ)] = 0
            F_x = np.sum(dF_x * dr) * numblades
            F_y = np.sum(dF_y * dr) * numblades
            F_z = np.sum(dF_z * dr) * numblades
            Q = np.sum(dQ * dr * r) * numblades
            # This is a temporary section that I stuck in for testing!  Calculating sideforce based on a 20-foot tail boom lever arm
            Fy_tailrotor = Q / 20.
            # calculate the tip-path-plane angles
            alpha_TPP = math.tan(Fx/Fz) # tip path plane forward angle
            phi_TPP = math.tan(Fy_tailrotor/Fz)
            # define angles relative to TPP and recalculate trim angles (no flapping, only feathering)
            beta_1c = 0
            beta_1s = 0
            theta_1s = alpha_TPP
            theta_1c = - phi_TPP
            if debug: pvar(locals(), ('F_x', 'F_y', 'F_z'))



    
    def trimOld(self, tolerance, cttarget, mu, numblades):
        ct = cttarget
        diskArea = math.pi * self.blade.radius**2
        bladeArea = np.sum(self.blade.chord*self.blade.radius * self.blade.dr * numblades)
        solidity = bladeArea / diskArea
        print solidity

        lock = self.lockNumber
        twist = self.twist
        
        steps = 0
        bladeLoading = ct / sigma
        
        # Alpha and inflow
        alpha = np.arctan(mu**2 * 0.015 / (2*ct))
        lambd = ct / (2.*mu)
        lastLambda = 9999999.;
        while abs(lambd - lastLambda) > tolerance:
            lastLambda = lambd
            lambd = mu * np.tan(alpha) + ct / (2 * np.sqrt(mu**2 + lambd**2))
        lambda_i = lambd
        # get guesses for angles
        b0 = .1
        t0 = .1
        t1c = .1
        t1s = .1
        last = 99999.
        while abs(last - (b0+t0+t1c+t1s)) > tolerance:
            last = b0 + t0 + t1c + t1s
            b0 = lock * (t0/8.*(1+mu**2) + twist/10*(1+5/6*mu**2) + mu/6*t1s - lambd/6)
            t1c = 4./3*mu*b0 / (1.+1./2*mu**2)
            t1s = -8./3*mu*(t0 - 3./4*lambd + 3./4*mu*t1s + 3./4*twist) / (1.-1./2*mu**2)
            t0 = 3.*(2.*ct/(sigma*a) - twist/4.*(1.+mu**2) - mu/2.*t1s + lambd/2.) / (1+3./2*mu**2)
        ## Find L' at all points
        # Perturb the values slightly so that the loop has something to go on
        L = .5 * (t0*x**2 + 2./mu*x*t0*np.sin(psi) + t0/mu*np.sin(psi)**2 + twist*x**3 + 2./mu*twist*x**2*np.sin(psi) + 
                twist/mu**2*x*np.sin(psi)**2 + x**2*t1c*np.cos(psi) + 2.*x/mu*t1c*np.sin(psi)*np.cos(psi) + 
                t1c/mu**2*np.sin(psi)**2*np.cos(psi) + 
                x**2*t1s*np.sin(psi) + 2./mu*x*t1s*np.sin(psi)**2 + t1s/mu**2*np.sin(psi)**3 - 
                x*lambd - lambd/mu*np.sin(psi) - b0/mu*x**np.cos(psi) - b0/mu**2*np.cos(psi)*np.sin(psi))
        L[~np.isfinite(L)] = 0
        b0 = lock/(2*math.pi) * np.sum(np.sum(L*x*rrstep)*psiistep)
        roll = a/(2*math.pi**2)*np.sum(np.sum(L*x*np.sin(psi)*rrstep)*psiistep)
        pitch = a/(2*math.pi**2)*np.sum(np.sum(L*x*np.cos(psi)*rrstep)*psiistep)
        ct = a*sigma/(2*math.pi)*np.sum(np.sum(L*rrstep)*psiistep)
        t0 = t0 + (cttarget - ct)
    
        # Loop until the rotor is trimmed
        dct = ct-cttarget
        if debug: pvar(locals(), ('dct', 'pitch', 'roll', 'cd'))
        while ((abs(ct - cttarget) > tolerance) or (abs(pitch) > tolerance) or (abs(roll) > tolerance) ) and (abs(ct) < 100):
            L = .5 * (t0*x**2 + 2./mu*x*t0*np.sin(psi) + t0/mu*np.sin(psi)**2 + twist*x**3 + 2./mu*twist*x**2*np.sin(psi) + 
                twist/mu**2*x*np.sin(psi)**2 + x**2*t1c*np.cos(psi) + 2*x/mu*t1c*np.sin(psi)*np.cos(psi) + 
                t1c/mu**2*np.sin(psi)**2*np.cos(psi) + 
                x**2*t1s*np.sin(psi) + 2./mu*x*t1s*np.sin(psi)**2 + t1s/mu**2*np.sin(psi)**3 - 
                x*lambd - lambd/mu*np.sin(psi) - b0/mu*x**np.cos(psi) - b0/mu**2*np.cos(psi)*np.sin(psi))
            atpp = np.arctan(mu**2*0.015/2/ct)
            l = lambda_i + mu*np.sin(atpp)
            lambd = lambda_i * (1. + (4./3*mu/l)/(1.2+mu/l)*x*np.cos(psi))
            ut = x + mu*np.sin(psi)
            alpha = (x*(t0 + twist*x + t1c*np.cos(psi) + t1s*np.sin(psi))
                + mu*((t0+twist*x)*np.sin(psi) + t1c*np.cos(psi)*np.sin(psi)
                + t1s*np.sin(psi)**2 - b0*np.cos(psi)) - lambd)
            L = alpha*0.5*ut
            b0 = lock/(2*math.pi) * np.sum(np.sum(L*x*rrstep)*psiistep)
            roll = a/(2*math.pi**2)*np.sum(np.sum(L*x*np.sin(psi)*rrstep)*psiistep)
            pitch = a/(2*math.pi**2)*np.sum(np.sum(L*x*np.cos(psi)*rrstep)*psiistep)
            ct = a*sigma/(2*math.pi)*np.sum(np.sum(L*rrstep)*psiistep)
            t0 = t0 + (cttarget - ct)
            t1s = t1s + pitch
            t1c = t1c - roll
            if debug: print 'b0: %f     ct: %f     t0: %f     roll: %f     t1c: %f     pitch: %f     t1s: %f\n' % (b0*180/math.pi, ct, t0*180/math.pi, roll, t1c*180/math.pi, pitch, t1s*180/math.pi)
            cthistory.append(ct)
            steps = steps + 1
        if debug: print 'end\n'

    def plot(self):
        pass

    
if __name__ == '__main__':
    rho = 0.001207 # slugs per ft^3
    V = 135. # feet per second
    f = 15. # square feet
    Vtip = 650. # ft/s
    mu = V / Vtip
    r = 30. # feet
    omega = Vtip / r # rad/s
    b = Blade(c81File='Config/S809_Cln.dat', skip_header=12, skip_footer=1, rootChord=1./30, taperRatio=.8, tipTwist=-12.*math.pi/180, rootCutout=.2, segments=15)
    r = Rotor(b, psiSegments=15, omega=omega, radius=r, pitch=5.*math.pi/180, numblades=4)
    Fhorizontal = 1./2 * rho * V**2 * f
    Fvertical = 15000. # pounds
    r.trim(tolerance=.1, V=V, rho=rho, speedOfSound=1026., Fx=Fhorizontal, Fy=0, Fz=Fvertical)