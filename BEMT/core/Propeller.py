import math

from core.Section import Section, Section_unlinear

class Propeller:
    
    """
    So the propeller is defined by:
    number of blades
    diameter
    a wing broken down into x elements (dr)
    A radius
    A chord
    A pitch
    A profil
    """

    def __init__(self, rotor_diameter, blade_count, section_count, radii, chords, pitches, profile, rho, unlinear : bool = False, azimuth : float = 0, a_disk : float = 0):
        
        self.rotor_diameter = rotor_diameter
        self.blade_count = blade_count
        self.section_count = section_count
        self.radii = radii
        self.chords = chords
        self.pitches = pitches
        self.rho = rho

        self.sections = []

        self.alpha = [float(p) for p in self.pitches]

        self.unlinear = unlinear
        self.azimuth = azimuth
        self.a_disk = a_disk

        # Check if radius are OK
        if any(r < 0 for r in self.radii):
            raise ValueError("Negative values for radius.")
        if not (len(self.radii) == len(self.chords) == len(self.pitches) == section_count):
            raise ValueError("radii, chords and pitches must match section_count.")

        if isinstance(profile, list):
            if len(profile) != section_count:
                raise ValueError("Profile list size must match section_count.")
            section_profiles = profile
        else:
            section_profiles = [profile] * section_count

        for i in range(section_count):
            if section_count == 1:
                width = radii[0]
            elif i == 0:
                width = radii[1] - radii[0]
            else:
                width = radii[i] - radii[i-1]
            width = abs(width)

            if unlinear:
                section = Section_unlinear(
                    profile=section_profiles[i],
                    radius=radii[i],
                    width=width,
                    pitch=math.radians(pitches[i]),  # Conversion en radians
                    chord=chords[i],
                    propeller=self,
                    azimuth=azimuth,  
                    a_disk=a_disk  
                )
            else:
                section = Section(
                    profile=section_profiles[i],
                    radius=radii[i],
                    width=width,
                    pitch=math.radians(pitches[i]),
                    chord=chords[i],
                    propeller=self,
                )
            self.sections.append(section)
