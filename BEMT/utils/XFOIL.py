import os
import subprocess
import numpy as np
from pathlib import Path
import tempfile
import warnings

class XFOIL_Runner:
    """
    Génère une polaire pour un airfoil.
    """
    def __init__(self, airfoil_name):
        self.airfoil_name = airfoil_name
        self.running_name = airfoil_name
        """ Running name = nom de la simulation, par défaut le NACA"""

        self.a_i = 0 # Alpha initial
        
        self.a_f = 0 # Alpha final

        self.step = 0 # Pas d'échantillonnage
        """ Pas d'échantillonnage"""
        
        self.Re = 0 # Nombre de Reynolds (turbulent)
        """ Nombre de Reynolds (turbulent) """

        self.iter = 100 # 
        """ Nombre d'itération (par défaut 100) """

        self.XCRIT = 0 ## 
        """ Point de bascule entre laminaire et turbulant. """
    def __check__(self):
        """
        On vérifie que les données sont entrées.
        """
        return not any(x == 0 for x in [self.a_f, self.step, self.Re, self.iter, self.XCRIT])
    
    def run(self, chord):
        # Vérification des paramètres
        if not self.__check__():
            print("[XFOIL_Runner] Erreur d'initialisation dans les input.")
            return
        
        if os.path.exists("polar_file.txt"):
            os.remove("polar_file.txt")
        
        # Création du fichier d'entrée pour XFOIL (lecture fichier + fonctions utilisées pour réaliser une polaire)
        # ATTENTION, les espaces sont nécessaires !!!!
        # sous load
        #GDES
        #SCAL {chord/0.026}VPAR


        input_commands = f"""
LOAD {self.airfoil_name}.dat
{self.running_name}

PANE
OPER
Visc {self.Re}
VPAR
XTR {self.XCRIT} {self.XCRIT}
N 5

PACC
polar_file.txt

ITER {self.iter}
ASeq {self.a_i} {self.a_f} {self.step}


quit
"""

        with open("input_file.in", 'w') as input_file:
            input_file.write(input_commands)
        
        # lecture xfoil
        xfoil_path = "xfoil.exe"  # S'assurer d'avoir le xfoil.exe et spécifier le path si fichier pas dans root source
        if not os.path.exists(xfoil_path):
            print(f"[XFOIL_Runner] Le fichier {xfoil_path} est introuvable.")
            return
        
        try:
            with subprocess.Popen(xfoil_path, stdin=subprocess.PIPE) as proc:
                proc.communicate(input=input_commands.encode())

            # Vérification de la création du fichier de polaire
            if os.path.exists("polar_file.txt"):
                polar_data = np.loadtxt("polar_file.txt", skiprows=12)
                return polar_data
            else:
                print("[XFOIL_Runner] Erreur : Le fichier polar_file.txt n'a pas été créé.")
        
        except Exception as e:
            print(f"[XFOIL_Runner] Erreur lors du lancement de XFOIL : {e}")


class XFOILPolarGenerator:
    """
    Generate a clean 3-column polar file (AoA, Cl, Cd) from airfoil points.
    """

    def __init__(self, xfoil_executable: str | Path | None = None, temp_root: str | Path | None = None):
        if xfoil_executable is None:
            this_dir = Path(__file__).resolve().parent
            xfoil_executable = this_dir / "xfoil.exe"
        self.xfoil_executable = Path(xfoil_executable).resolve()
        if temp_root is None:
            env_tmp = os.getenv("BEMT_XFOIL_TMP")
            if env_tmp:
                temp_root = Path(env_tmp)
            else:
                temp_root = Path.cwd() / "outputs" / "xfoil_tmp"
        self.temp_root = Path(temp_root).resolve()

    def _check_executable(self):
        if not self.xfoil_executable.exists():
            raise FileNotFoundError(f"XFOIL executable not found: {self.xfoil_executable}")

    @staticmethod
    def _downsample_points(points, max_points: int = 999):
        if len(points) <= max_points:
            return points

        stride = (len(points) - 1) / (max_points - 1)
        sampled = []
        for i in range(max_points):
            idx = int(round(i * stride))
            idx = min(idx, len(points) - 1)
            sampled.append(points[idx])
        return sampled

    def generate_from_points(
        self,
        points_xy,
        reynolds: float,
        output_file: str | Path,
        aoa_start: float = 0,
        aoa_end: float = 20.0,
        aoa_step: float = 0.5,
        iterations: int = 100,
        xcrit: float = 9.0,
        max_points: int = 999,
        timeout_seconds: int = 120,
    ):
        self._check_executable()
        env_timeout = os.getenv("BEMT_XFOIL_TIMEOUT")
        if env_timeout:
            try:
                timeout_seconds = float(env_timeout)
            except ValueError:
                pass

        points = self._downsample_points(list(points_xy), max_points=max_points)
        if len(points) < 3:
            raise ValueError("At least 3 points are required to run XFOIL.")

        output_path = Path(output_file).resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        self.temp_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="bemt_xfoil_", dir=self.temp_root) as tmp:
            tmp_dir = Path(tmp)
            airfoil_file = tmp_dir / "airfoil.dat"
            polar_file = tmp_dir / "polar_raw.txt"

            with airfoil_file.open("w", encoding="utf-8") as handle:
                handle.write("AIRFOIL\n")
                for x, y in points:
                    handle.write(f"{float(x):.8f} {float(y):.8f}\n")

            attempts = [
                (aoa_start, aoa_end, aoa_step),
                (-10.0, 15.0, 0.5),
                (-5.0, 12.0, 0.5),
                (-2.0, 10.0, 0.5),
            ]

            last_details = ""
            for try_a_start, try_a_end, try_a_step in attempts:
                if polar_file.exists():
                    polar_file.unlink()

                commands = f"""
LOAD {airfoil_file.name}

PANE
PPAR
N
70


OPER
Visc {float(reynolds):.1f}
VPAR
XTR {float(xcrit):.3f} {float(xcrit):.3f}
N 5

PACC
{polar_file.name}

ITER {int(iterations)}
ASeq {float(try_a_start):.3f} {float(try_a_end):.3f} {float(try_a_step):.3f}
PACC

QUIT
"""
                proc = subprocess.run(
                    [str(self.xfoil_executable)],
                    input=commands,
                    text=True,
                    cwd=tmp_dir,
                    capture_output=True,
                    timeout=timeout_seconds,
                )

                details = (proc.stdout or "")[-800:] + "\n" + (proc.stderr or "")[-800:]
                last_details = details.strip()

                if not polar_file.exists():
                    continue

                if polar_file.stat().st_size == 0:
                    continue

                try:
                    with warnings.catch_warnings():
                        warnings.simplefilter("ignore", category=UserWarning)
                        polar_data = np.loadtxt(polar_file, skiprows=12, usecols=(0, 1, 2))
                except Exception:
                    continue

                polar_data = np.atleast_2d(polar_data)
                if polar_data.size == 0:
                    continue

                np.savetxt(output_path, polar_data, fmt="%.6f")
                return output_path

            raise RuntimeError(
                "XFOIL did not produce a usable polar file after multiple AoA sweep attempts.\n"
                + last_details
            )

        return output_path
