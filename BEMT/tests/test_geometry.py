from pathlib import Path

import pytest

from core.Geometry import Geometry
from core.JSONReader import JSONReader
from core.Simulation import SingleProp
from core.SimulationParameters import SimulationParameters


def test_json_reader_unit_conversion_and_point_limit():
    reader = JSONReader("geometries/Mejzlik/VX4_hover_5_blades_50_sections.json", unit="mm")
    strips = reader.read_strips(max_points=1000)

    assert len(strips) > 0
    assert strips[0].radius_m == pytest.approx(0.027496210193634, rel=1e-8)
    assert strips[0].radius_m < 1.0
    assert len(strips[0].points_xy) <= 1000


def test_geometry_from_json():
    geometry = Geometry.from_json("Mejzlik", unit="mm")
    assert geometry.section_count > 0
    assert geometry.radii[0] < 1.0
    assert len(geometry.sections[0].points_xy) <= 1000


def test_singleprop_from_geometry_with_shared_polar():
    geometry = Geometry.from_manual(
        name="X500",
        radii=[0.013, 0.026, 0.039],
        chords=[0.016, 0.017, 0.024],
        pitches=[9, 30, 25],
        profiles="HQ_2012",
    )
    geometry.set_shared_polar(Path("geometries/X500/Polars/HQ_2012.dat"))

    params = SimulationParameters(
        v_inf=0.0,
        rpm=6000.0,
        blade_count=2,
        rotor_diameter=0.26,
    )
    rotor = SingleProp.from_geometry(
        geometry=geometry,
        simulation=params,
        generate_missing_polars=False,
    )

    thrust, torque, power = rotor.calculate()
    assert thrust > 0.0
    assert torque > 0.0
    assert power > 0.0
