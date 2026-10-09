
# On va définir tous les éléments nécessaires pour réaliser la BEMT

import math
import numpy as np

from scipy.optimize import minimize
    
class Section:

    """
    Assumptions of BEMT:
    
    - No aerodynamic interaction between blade elements
    - Forces are only determined by lift and drag coefficients
    
    Input parameters of the section:

    A radius = Distance between the center and the middle of the section,
    A width = width of the section (depends on sampling step),
    A pitch = an angle in radians of inclination,
    A chord length = length of the section chord
    The associated propeller

    To be determined:
    Angle of attack (alpha),
    Lift coefficient (Cl)
    Drag coefficient (Cd)
    """
    def __init__(self, profile, radius, width, pitch, chord, propeller):

        self.profile = profile

        self.radius = radius
        self.width = width
        self.pitch = pitch
        self.chord = chord

        self.alpha = 0.0
        self.Cl = 0.0
        self.Cd = 0.0
        self.phi = 0.0 # Angle inflow
        self.propeller = propeller
        self.current_re: float = 1e5  # updated after each forces() call

        # Sigma = rotor solidity
        # Video : https://www.youtube.com/watch?v=_CVnIZCTfRE&t=1511s
        self.sigma = self.propeller.blade_count * self.chord / (2*math.pi*self.radius)

        self.T = 0
        self.Q = 0
        self.P = 0
        
        self.Re = 0

        self.va = 0
        self.vt = 0

    def airfoil_section_forces(self, phi, re: float | None = None):
        self.alpha = (self.pitch-phi) # Angle of attack = pitch minus phi (inflow angle)

        self.aoa = math.degrees(self.alpha)

        effective_re = re if re is not None else self.current_re
        self.Cl = self.profile.getCl(self.alpha, re=effective_re)
        self.Cd = self.profile.getCd(self.alpha, re=effective_re)

        """
        Force coefficients on the section profile:
        Thrust coefficient: CT = Cl * cos(phi) - Cd * sin(phi)
        Torque coefficient: CQ = Cl * sin(phi) + Cd * cos(phi)
        """
        CT = self.Cl*math.cos(phi)-self.Cd*math.sin(phi)
        CQ = self.Cl*math.sin(phi)+self.Cd*math.cos(phi)

        self.CT = CT
        self.CQ = CQ

        return CT, CQ
    
    """
    Prandtl's loss factor: simplifies rotor wake effects
    Links in thesis
    """
    def prandtl_loss_factor(self, phi):

        """
        Formula found on the internet. This involves calculating a coefficient F which, depending on the radius, makes it possible to determine the tip loss and 
        include it in our final calculation
        """

        radius = self.radius
        radius_rotor = self.propeller.rotor_diameter/2
        step = radius_rotor-radius
        self.step = step
        func = self.propeller.blade_count*step/(2*radius*(math.sin(phi)))
        func = min(700, func) 
        func = max(-700, func)

        F = 2*math.acos(min(1.0, math.exp(-func)))/math.pi

        return F
    
    """
    From Froude's Theory relating to capture propellers (Momentum Theory), we can make a relationship
    between the axial induced speed and the thrust force.

    To determine it, we need to determine its factors, also called induced factors.
    """
    def froude_induction_factors(self, phi):

        CT, CQ = self.airfoil_section_forces(phi)
        F = self.prandtl_loss_factor(phi)
        F = max(F, 1e-8)
        
        X = 4*F*math.sin(phi)**2/(self.sigma * CT)
        Y = (4*F*math.sin(phi)*math.cos(phi))/(self.sigma * CQ)

        facteur_induction_axial = 1/(X-1)
        facteur_induction_translationnel = 1/(Y+1)

        return facteur_induction_axial, facteur_induction_translationnel
    
    """
    We have all the elements of the BEMT combined to determine the thrust and the torque on the section

    We just need to calculate the local velocities (axial and tangential)
    """

    def phi_function(self, phi, omega, v_inf, azimuth: float = 0, a_disk: float = 0):
        try:
            CT, CQ = self.airfoil_section_forces(phi)
            F = self.prandtl_loss_factor(phi)
            F = max(F, 1e-8)

            sin_phi = math.sin(phi)
            cos_phi = math.cos(phi)

            if abs(sin_phi) < 1e-12:
                return 100.0
            
            Y = 4.0 * F * sin_phi * cos_phi / (self.sigma * CQ)
            t = 1.0 / (Y + 1.0)

            denom = omega * self.radius * (1.0 - t)
            if abs(denom) < 1e-12:
                return 100.0

            f = sin_phi - self.sigma * CT / (4.0 * F * sin_phi) - v_inf * cos_phi / denom
        except (ZeroDivisionError, ValueError):
            f = 100.0
        return f
    
    def forces(self, phi, v_infini, omega):

        rho = self.propeller.rho
        self.phi = phi

        CT, CQ = self.airfoil_section_forces(phi)
        F = self.prandtl_loss_factor(phi)
        F = max(F, 1e-8)

        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        Y = 4.0 * F * sin_phi * cos_phi / (self.sigma * CQ)
        t = 1.0 / (Y + 1.0)

        vt = (1.0 - t) * omega * self.radius
        va = vt * math.tan(phi) if abs(cos_phi) > 1e-8 else v_infini
        V = math.sqrt(va**2 + vt**2)

        mu = 1.81e-5
        Re = rho * V * self.chord / mu if V > 0 else self.current_re
        self.Re = Re

        # Re-evaluate Cl/Cd at the actual operating Re (Re-scaling in Profil.getCd)
        if abs(Re - self.current_re) / max(self.current_re, 1.0) > 0.01:
            CT, CQ = self.airfoil_section_forces(phi, re=Re)
            self.current_re = Re

        self.CT = CT

        self.T = self.sigma * math.pi * rho * V**2 * CT * self.width * self.radius
        self.Q = self.sigma * math.pi * rho * V**2 * CQ * self.width * self.radius**2

        return self.T, self.Q


"""
This is the version including the alfa disk (the disk angle of attack).

a_disk is basically the angle between axis of rotation and the free steam velocity. It will logically be limited between 0 and 90° for our BEMT

azimuth (Ψ) will be the azimuthal position of the blade. It will be equal to 0 when the blade radius lies on positive Z_propeller axis !
In that case, Ψ will either increase or decrease (depending on the convention used) for a rotation in Z_propeller direction.

"""

class Section_unlinear(Section):

    def __init__(self, profile, radius, width, pitch, chord, propeller, azimuth, a_disk):

        super().__init__(profile, radius, width, pitch, chord, propeller)

        self.azimuth = azimuth
        self.a_disk = a_disk
        self.w_upwash = 0.0   # axial velocity perturbation from strut potential flow (m/s)

        # set values to 0
        # self.alpha = 0.0
        # self.Cl = 0.0
        # self.Cd = 0.0
        # self.phi = 0.0
        
        # self.sigma = self.propeller.nb_blades * self.chord / (2 * math.pi * self.radius) # Rotor solidity is the same

        # self.T = 0
        # self.Q = 0

        self.section =  Section(profile, radius, width, pitch, chord, propeller)
    
    def _induction_and_velocities(self, phi, omega, v_inf, azimuth, a_disk, CQ, F):
        """Tangential induction a' and local velocities for oblique inflow.

        Velocity triangle at the blade element (azimuth Ψ, disk angle a_disk):
            tangential (before induction): U_t0 = Ωr + V∞ sin(a_disk) sin(Ψ)
            axial freestream:              V_a0 = V∞ cos(a_disk) + w_upwash

        Momentum balance uses Glauert's mass flux U = sqrt(va² + V_par²):
        the in-plane freestream component V_par = V∞ sin(a_disk) increases the
        mass flow through the annulus and lowers the induced velocity
        (translational lift). a' is refined by fixed-point since U depends on it.
        """
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        V_par = v_inf * math.sin(a_disk)
        U_t0 = omega * self.radius + V_par * math.sin(azimuth)
        V_a0 = v_inf * math.cos(a_disk) + self.w_upwash

        sigma_CQ = self.sigma * CQ
        if abs(sigma_CQ) < 1e-12 or abs(cos_phi) < 1e-8:
            t = 0.0
            vt = U_t0
            va = V_a0
            U = math.sqrt(va * va + V_par * V_par)
            return t, vt, va, U, U_t0, V_a0

        Y = 4.0 * F * sin_phi * cos_phi / sigma_CQ
        t = 1.0 / (Y + 1.0)
        for _ in range(3):
            vt = (1.0 - t) * U_t0
            va = vt * math.tan(phi)
            U = math.sqrt(va * va + V_par * V_par)
            if va < 1e-9 or U < 1e-9:
                break
            t = 1.0 / (Y * (U / va) + 1.0)

        vt = (1.0 - t) * U_t0
        va = vt * math.tan(phi)
        U = math.sqrt(va * va + V_par * V_par)
        return t, vt, va, U, U_t0, V_a0

    """
    phi_function and forces both change: the in-plane freestream component enters
    the blade velocity triangle AND the momentum mass flux (Glauert).
    """
    def phi_function(self, phi, omega, v_inf, azimuth, a_disk):
        """ phi =  arctan V∞ cos a_disk(1 + a)/(V∞sin a_disk sin Ψ + Ωr)(1 - a')"""
        try:
            CT, CQ = self.airfoil_section_forces(phi)
            F = self.prandtl_loss_factor(phi)
            F = max(F, 1e-8)

            sin_phi = math.sin(phi)
            cos_phi = math.cos(phi)

            if abs(sin_phi) < 1e-12:
                return 100.0

            t, vt, va, U, U_t0, V_a0 = self._induction_and_velocities(
                phi, omega, v_inf, azimuth, a_disk, CQ, F
            )

            denom = U_t0 * (1.0 - t)
            if abs(denom) < 1e-12:
                return 100.0

            # Glauert mass-flux ratio: va/U = 1 in pure axial flow (original BEMT),
            # < 1 in oblique flow -> less induced velocity for the same thrust.
            ratio = va / U if U > 1e-9 else 1.0
            f = sin_phi - (self.sigma * CT / (4.0 * F * sin_phi)) * ratio \
                - V_a0 * cos_phi / denom
        except (ZeroDivisionError, ValueError):
            f = 100.0
        return f

    def forces(self, phi, v_infini, omega):
        """Same as Section.forces but with the oblique-inflow velocity triangle,
        so the dynamic pressure seen by the blade is azimuth/a_disk consistent
        with phi_function (advancing/retreating blade asymmetry)."""
        rho = self.propeller.rho
        self.phi = phi

        CT, CQ = self.airfoil_section_forces(phi)
        F = self.prandtl_loss_factor(phi)
        F = max(F, 1e-8)

        t, vt, va, U, U_t0, V_a0 = self._induction_and_velocities(
            phi, omega, v_infini, self.azimuth, self.a_disk, CQ, F
        )
        V = math.sqrt(va**2 + vt**2)

        mu = 1.81e-5
        Re = rho * V * self.chord / mu if V > 0 else self.current_re
        self.Re = Re

        # Re-evaluate Cl/Cd at the actual operating Re (Re-scaling in Profil.getCd)
        if abs(Re - self.current_re) / max(self.current_re, 1.0) > 0.01:
            CT, CQ = self.airfoil_section_forces(phi, re=Re)
            self.current_re = Re

        self.CT = CT
        self.CQ = CQ
        self.va = va
        self.vt = vt

        self.T = self.sigma * math.pi * rho * V**2 * CT * self.width * self.radius
        self.Q = self.sigma * math.pi * rho * V**2 * CQ * self.width * self.radius**2

        return self.T, self.Q


    
