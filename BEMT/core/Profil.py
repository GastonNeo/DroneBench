from scipy.interpolate import interp1d
import math

class Profil:

    """
    This class will determine the Cl and Cd for a given angle of attack.

    If we have a lot of data, we can interpolate (which is much more convenient).

    Temporarily, we will use manual data.

    We assume that the data is structured as follows:

    Angle of attack, Lift coef, Drag coef

    re_ref: Reynolds number at which this polar was computed (optional).
    When provided, getCd applies a turbulent BL scaling Cd ∝ Re^(-0.2) so that
    operating at a different Re gives a physically corrected drag coefficient.
    """
    def __init__(self, donnees, re_ref: float | None = None):

        self.donnees = donnees
        self.re_ref = re_ref
        self.Cl_interp = None
        self.Cd_interp = None

        self.calcul()

    def calcul(self):
        alpha = self.donnees[0]
        cl = self.donnees[1]
        cd = self.donnees[2]

        # Outside the tabulated AoA range, keep edge values instead of extrapolating.
        self.Cl_interp = interp1d(
            alpha,
            cl,
            bounds_error=False,
            fill_value=(cl[0], cl[-1]),
        )
        self.Cd_interp = interp1d(
            alpha,
            cd,
            bounds_error=False,
            fill_value=(cd[0], cd[-1]),
        )

    def angle_normalise(self, alpha):
        return (alpha + math.pi) % (2 * math.pi) - math.pi

    def getCl(self, alpha, re: float | None = None):
        Cl = self.Cl_interp(math.degrees(self.angle_normalise(alpha)))
        return float(Cl)

    def getCd(self, alpha, re: float | None = None):
        Cd = self.Cd_interp(math.degrees(self.angle_normalise(alpha)))
        Cd = max(float(Cd), 0.0)
        # Turbulent BL Re-scaling: Cd ∝ Re^(-0.2)
        if self.re_ref is not None and re is not None and re > 0:
            Cd *= (self.re_ref / re) ** 0.2
        return Cd
