import pytest
import os
import numpy as np

from pathlib import Path
from core.Propeller import Propeller
from core.Profil import Profil

# Load airfoil profil
directory = os.path.dirname(os.path.abspath(__file__))
parent_directory = os.path.abspath(os.path.join(directory, ".."))

file_path = os.path.join(parent_directory, "HQ_2012.dat")
hq2012_alpha, hq2012_cl, hq2012_cd = np.loadtxt(file_path, unpack=True)
profil_hq2012 = Profil([hq2012_alpha, hq2012_cl, hq2012_cd])

def test_propeller_creation():

    prop = Propeller(0.26, 2, 10, [1,2,3,4,5,6,7,8,9,10], [1,2,3,4,5,6,7,8,9,10], [1,2,3,4,5,6,7,8,9,10], profil_hq2012, 1.225)
    assert prop.blade_count

def test_propeller_negative_radius():
    with pytest.raises(ValueError):
        Propeller(0.26, 2, 10, [-1,-2,-3,-4,-5,-6,-7,-8,-9,-10], [1,2,3,4,5,6,7,8,9,10], [1,2,3,4,5,6,7,8,9,10], profil_hq2012, 1.225)