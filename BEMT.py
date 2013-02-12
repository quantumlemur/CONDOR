



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







    import math as m
    import numpy as np
    from scipy import interpolate

    debug = True

    def pvar(locals_, vars_):
        s = ['%s: %d' % (var, locals_[var]) for var in vars_]
        print '     '.join(s)


    class Blade:
        
        def __init__(self, c81File, radius, chord, skip_header=0, skip_footer=0, taper=0, twist=0, cutout=0, pitch=0, segments=15):
            self.radius = radius
            airfoildata = np.genfromtxt(c81File, skip_header=skip_header, skip_footer=skip_footer) # read in the airfoil file
            # generate the airfoil data splines.  They are assumed to be in degrees, and are converted to radians
            self.clspl = interpolate.splrep(airfoildata[:,0]*m.pi/180, airfoildata[:,1])
            self.cdspl = interpolate.splrep(airfoildata[:,0]*m.pi/180, airfoildata[:,2])
            self.cmspl = interpolate.splrep(airfoildata[:,0]*m.pi/180, airfoildata[:,3])
            # generate the piecewise data on the blade
            endpoints, self.dr = np.linspace(cutout, 1, segments+1, retstep=True)
            self.r = np.zeros(endpoints.size - 1)
            for i in range(self.r.size):
                self.r[i] = (endpoints[i] + endpoints[i+1]) / 2
            twistspl = interpolate.splrep(np.array([cutout,1])*radius, np.array([0,twist]), k=1)
            self.twist = interpolate.splev(self.r, twistspl)
            chordspl = interpolate.splrep(np.array([cutout,1])*radius, np.array([chord,chord*(1-taper)]), k=1)
            self.chord = interpolate.splev(self.r, chordspl)
            # set the pitch
            self.setPitch(pitch)
        
        def cl(self, alpha):
            return interpolate.splev(alpha, self.clspl)
        
        def cd(self, alpha):
            return interpolate.splev(alpha, self.cdspl)
        
        def cm(self, alpha):
            return interpolate.splev(alpha, self.cmspl)
        
        def a(self, alpha):
            return interpolate.splev(alpha, self.clspl, der=1)
        
        def setPitch(self, pitch):
            self.pitch = pitch
            self.theta = pitch + self.twist
        
        def write(self):
            np.savetxt('Config/Blade.csv', np.transpose((self.r, self.theta)))
            alphas = np.arange(15)
            cl = self.cl(alphas)
            cd = self.cd(alphas)
            cm = self.cm(alphas)
            np.savetxt('Config/Airfoil.csv', np.transpose((alphas, cl, cd, cm)))


    class Rotor:
        
        def __init__(self, blade, psiSegments, omega):
            self.blade = blade
            self.omega = omega
            # set up the 2D arrays for psi, r, and L
            psi1D, psistep = np.linspace(0, 2*m.pi, psiSegments, endpoint=False, retstep=True)
            r1D = self.blade.r
            psi = np.zeros((psi1D.size, r1D.size))
            theta = np.zeros((psi1D.size, r1D.size))
            r = np.zeros((psi1D.size, r1D.size))
            for i in range(r1D.size):
                psi[:,i] = psi1D
            for i in range(psi1D.size):
                r[i,:] = r1D
                theta[i,:] = blade.theta
                chord[i,:] = blade.chord
            Cl = np.zeros((psi1D.size, r1D.size))
            self.psi1D = psi1D
            self.r1D = r1D
            self.psi = psi
            self.r = r
            self.theta = theta
            self.chord = chord
            print psi
            self.Cl = Cl
        
        def setPitch(self, pitch):
            self.blade.setPitch(pitch)
            for i in range(self.r1D.size):
                self.theta[i,:] = self.blade.theta
        
        def BEM(self, tolerance, mu):
            psi = self.psi
            r = self.r
            theta = self.theta
            chord = self.chord
            omega = self.omega
            R = self.blade.radius
            dr = self.blade.dr
            
            # initialize the inflow, assuming uniform plus tip losses
            Clalpha = self.blade.a(alpha)
            inflow = ct / (2. * solidity  # TODO: Finish writing equation from p.104
            
            
            
            betadot = 0 # we're going to assume no flapping
            beta = 5. * m.pi/180 # assume a 5deg precone for now.  Should this include alphaTPP and be a function of phi?
            
            # I'm dimensionalizing these three after all:
            Ut = omega*r + mu*omega*r*np.sin(psi) #r + mu*np.sin(psi) # nondimensionalized.  Is really Ut/(omega*R).  Leishman eq 3.162
            Up = inflow*omega*r + r*betadot + mu*omega*r*beta*np.cos(psi) #inflow + r*betadot/omega + mu*beta*np.cos(psi) # nondimensionalized.  Is really Up/(omega*R).  Leishman eq 3.163
            Ur = mu*omega*r*np.cos(psi) #mu*np.cos(psi) # nondimensionalized.  Is really Ur/(omega*R).  Leishman eq 3.164
            
            U = np.sqrt(Ut**2 + Up**2) # resultant velocity at blade element
            phi = np.atan(Up/Ut) # induced angle of attack
            alpha = theta - phi
            Clalpha = self.blade.a(alpha)
            dL = 1./2 * rho * chord * Clalpha * (theta * Ut**2 - Up*Ut) * dr  # should change this back to use Cl instead of Clalpha, since we can calculate it and don't need to assume constand Clalpha
            Cd = self.blade.cd(alpha)
            dD = 1./2 * rho * U**2 * chord * Cd * dr
            # now resolve those forces perpendicular and parallel to rotor disk:
            dFz = dL * np.cos(phi) - dD * np.sin*phi # can probably use a small-angle assumption to give dFz=dL
            dFx = dL*np.sin(phi) + dD*np.cos(phi) # can probably use a small-angle assumption to give dFx=phi*dL+dDs
            
            # stopped at page 116, on inflow models.  Also need to go back up and initialize the inflow before calculating the other stuff.  Should probably just put it at the top and iterate the whole thing.
        
        def trim(self, tolerance, cttarget, mu):
            ct = cttarget
            sigma = self.solidity
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
            b0 = lock/(2*m.pi) * np.sum(np.sum(L*x*rrstep)*psiistep)
            roll = a/(2*m.pi**2)*np.sum(np.sum(L*x*np.sin(psi)*rrstep)*psiistep)
            pitch = a/(2*m.pi**2)*np.sum(np.sum(L*x*np.cos(psi)*rrstep)*psiistep)
            ct = a*sigma/(2*m.pi)*np.sum(np.sum(L*rrstep)*psiistep)
            t0 = t0 + (cttarget - ct)
        
            # Loop until the rotor is trimmed
            print 'dct: %f     pitch: %f     roll: %f     cd: %f' % (ct-cttarget, pitch, roll, ct)
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
                b0 = lock/(2*m.pi) * np.sum(np.sum(L*x*rrstep)*psiistep)
                roll = a/(2*m.pi**2)*np.sum(np.sum(L*x*np.sin(psi)*rrstep)*psiistep)
                pitch = a/(2*m.pi**2)*np.sum(np.sum(L*x*np.cos(psi)*rrstep)*psiistep)
                ct = a*sigma/(2*m.pi)*np.sum(np.sum(L*rrstep)*psiistep)
                t0 = t0 + (cttarget - ct)
                t1s = t1s + pitch
                t1c = t1c - roll
                print 'b0: %f     ct: %f     t0: %f     roll: %f     t1c: %f     pitch: %f     t1s: %f\n' % (b0*180/m.pi, ct, t0*180/m.pi, roll, t1c*180/m.pi, pitch, t1s*180/m.pi)
                cthistory.append(ct)
                steps = steps + 1
            print 'end\n'

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
        b = Blade(c81File='Config/S809_Cln.dat', skip_header=12, skip_footer=1, radius=30., chord=1., twist=-12.*m.pi/180, pitch=0.*m.pi/180, cutout=.2, segments=15)
        r = Rotor(b, psiSegments=15, omega=omega)
        Fhorizontal = 1./2 * rho * V^2 * f
        Fvertical = 15000. # pounds
        r.BEM(tolerance=0.001, mu=mu, Fhorizontal=Fhorizontal, Fvertical=Fvertical)