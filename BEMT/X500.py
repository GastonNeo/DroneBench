import BEMT

res = BEMT.run_xfoil("X500_classic", json_file="x500_strips.json", rpm=6300, v_inf=0, blade_count=2, rotor_diameter=0.26, xcrit=1, unit="m")
print(res.thrust)

res = BEMT.run("X500_classic", json_file="x500_strips.json", rpm=6300, v_inf=0, blade_count=2, rotor_diameter=0.26, unit="m")
print(res.thrust)