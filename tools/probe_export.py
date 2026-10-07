#!/usr/bin/env python3
"""Build and step minimal quad/hexa FMI 2.0 CS models with OpenModelica (Linux)."""
import argparse
import ctypes as ct
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


class Fmi2CS:
    """Small FMI 2.0 Co-Simulation host; no fallback to native Modelica simulation."""

    def __init__(self, fmu: Path, directory: Path, n: int):
        self.inputs = [f"pwm_us[{i}]" for i in range(1, n+1)]
        self.observables = ([f"rotorSpeed[{i}]" for i in range(1, n+1)]
            + [f"{key}[{i}]" for key in ("position_ned", "velocity_ned", "acceleration_frd", "truthPosition_w", "truthVelocity_w") for i in range(1, 4)]
            + ["altitude_m", "climbRate_mps"])
        directory.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(fmu) as archive:
            for member in archive.infolist():
                require((directory / member.filename).resolve().is_relative_to(directory.resolve()),
                        "FMU contains a path outside the extraction directory")
            archive.extractall(directory)
        description = ET.parse(directory / "modelDescription.xml").getroot()
        require(description.attrib.get("fmiVersion") == "2.0", "Expected FMI 2.0")
        cs = description.find("CoSimulation")
        require(cs is not None, "FMU has no CoSimulation interface")
        self.identifier = cs.attrib["modelIdentifier"]
        self.variables = {
            variable.attrib["name"]: variable
            for variable in description.find("ModelVariables")
        }
        for name in self.inputs:
            require(name in self.variables and self.variables[name].attrib.get("causality") == "input",
                    f"Missing real PWM input {name}")
        for name in self.observables:
            require(name in self.variables and self.variables[name].attrib.get("causality") == "output",
                    f"Missing exported output {name}")
        binary = directory / "binaries" / "linux64" / f"{self.identifier}.so"
        require(binary.is_file(), f"No Linux 64-bit FMI binary at {binary}")
        self.library = ct.CDLL(str(binary))
        self.log: list[str] = []
        logger_type = ct.CFUNCTYPE(None, ct.c_void_p, ct.c_char_p, ct.c_int, ct.c_char_p, ct.c_char_p)

        def log_message(_environment, _instance, status, category, message):
            self.log.append(f"{status}:{(category or b'').decode(errors='replace')}:"
                            f"{(message or b'').decode(errors='replace')}")

        self.logger = logger_type(log_message)
        self.libc = ct.CDLL(None)
        self.libc.calloc.argtypes = [ct.c_size_t, ct.c_size_t]
        self.libc.calloc.restype = ct.c_void_p
        self.libc.free.argtypes = [ct.c_void_p]
        self.libc.free.restype = None

        class Callbacks(ct.Structure):
            _fields_ = [(name, ct.c_void_p) for name in (
                "logger", "allocateMemory", "freeMemory", "stepFinished", "componentEnvironment")]

        self.callbacks = Callbacks(ct.cast(self.logger, ct.c_void_p),
                                   ct.cast(self.libc.calloc, ct.c_void_p),
                                   ct.cast(self.libc.free, ct.c_void_p), None, None)
        self._bind("fmi2Instantiate", [ct.c_char_p, ct.c_int, ct.c_char_p, ct.c_char_p,
                                      ct.POINTER(Callbacks), ct.c_int, ct.c_int], ct.c_void_p)
        self._bind("fmi2SetupExperiment", [ct.c_void_p, ct.c_int, ct.c_double,
                                          ct.c_double, ct.c_int, ct.c_double])
        for name in ("fmi2EnterInitializationMode", "fmi2ExitInitializationMode", "fmi2Terminate"):
            self._bind(name, [ct.c_void_p])
        self._bind("fmi2FreeInstance", [ct.c_void_p], None)
        for name in ("fmi2SetReal", "fmi2GetReal"):
            self._bind(name, [ct.c_void_p, ct.POINTER(ct.c_uint), ct.c_size_t, ct.POINTER(ct.c_double)])
        self._bind("fmi2DoStep", [ct.c_void_p, ct.c_double, ct.c_double, ct.c_int])
        self.component = self.fmi2Instantiate(
            b"fire_export_probe", 1, description.attrib["guid"].encode(),
            ((directory / "resources").resolve().as_uri() + "/").encode(),
            ct.byref(self.callbacks), 0, 0)
        require(bool(self.component), "fmi2Instantiate failed")
        self.held_inputs = None

    def _bind(self, name: str, arguments: list, result=ct.c_int) -> None:
        function = getattr(self.library, name, None)
        if function is None:
            function = getattr(self.library, self.identifier + "_" + name)
        function.argtypes, function.restype = arguments, result
        setattr(self, name, function)

    def _check(self, status: int, operation: str) -> None:
        require(status <= 1, f"{operation} returned FMI status {status}: {self.log[-5:]}")

    def _references(self, names: list[str]):
        return (ct.c_uint * len(names))(*(int(self.variables[name].attrib["valueReference"]) for name in names))

    def set_inputs(self, values: list[float]) -> None:
        self._check(self.fmi2SetReal(self.component, self._references(self.inputs), len(self.inputs),
                                   (ct.c_double * len(values))(*values)), "fmi2SetReal")
        self.held_inputs = list(values)

    def initialize(self, values: list[float], stop: float) -> None:
        self._check(self.fmi2SetupExperiment(self.component, 1, 1e-8, 0, 1, stop), "setup")
        self.set_inputs(values)
        self._check(self.fmi2EnterInitializationMode(self.component), "enter initialization")
        self._check(self.fmi2ExitInitializationMode(self.component), "exit initialization")

    def snapshot(self) -> dict[str, float]:
        # OpenModelica 1.26.1 internalGetEventIndicators clears _need_update
        # after evaluating only the ODE. With contact events, algebraic sensor
        # outputs can therefore lag the states by one CS step. Reapply the
        # unchanged held inputs at this communication point to invalidate the
        # runtime cache before fmi2GetReal; no input value or state is changed.
        # A production host must handle this too, or use a corrected runtime.
        if self.held_inputs is not None:
            self.set_inputs(self.held_inputs)
        values = (ct.c_double * len(self.observables))()
        self._check(self.fmi2GetReal(self.component, self._references(self.observables), len(values), values), "get real")
        require(all(math.isfinite(value) for value in values), "Nonfinite exported real value")
        result = dict(zip(self.observables, values))
        return result

    def step(self, time: float, step: float) -> None:
        self._check(self.fmi2DoStep(self.component, time, step, 1), f"fmi2DoStep at t={time}, dt={step}")

    def close(self) -> None:
        if self.component:
            self.fmi2Terminate(self.component)
            self.fmi2FreeInstance(self.component)
            self.component = None

def run(output, omc):
    output.mkdir(parents=True, exist_ok=True)
    (output / "report.json").unlink(missing_ok=True)
    metrics = {}
    for preset, n in [("QuadX", 4), ("HexaX", 6)]:
        work = output / preset
        work.mkdir(exist_ok=True)
        harness = work / "ExportCopter.mo"
        harness.write_text('model ExportCopter\n  extends fire_modelica_models.Adapters.FastDyn.MultirotorFmu(\n'
            f'    geometry=fire_modelica_models.Vehicles.Copter.Presets.{preset}());\nend ExportCopter;\n')
        script = work / "export.mos"
        script.write_text('loadModel(Modelica,{"4.0.0"});\n'
            f'loadFile({json.dumps(str(ROOT / "package.mo"))});\n'
            f'loadFile({json.dumps(str(harness))});\n'
            'buildModelFMU(ExportCopter,version="2.0",fmuType="cs",fileNamePrefix="ExportCopter",platforms={"static"});\ngetErrorString();\n')
        fmu = work / "ExportCopter.fmu"
        fmu.unlink(missing_ok=True)  # Only this tool's generated artifact; reject stale exports.
        proc = subprocess.run([omc,str(script)],cwd=work,capture_output=True,text=True,timeout=300)
        log = proc.stdout + proc.stderr
        (work / "build.log").write_text(log)
        require(proc.returncode == 0 and "Error:" not in log and fmu.is_file(), f"Export failed: {work / 'build.log'}")
        host = Fmi2CS(fmu, work / "unpacked", n)
        dt = 0.0001
        try:
            host.initialize([1000]*n, 0.061)  # Leave margin for binary roundoff at the last doStep.
            for tick in range(200):
                host.step(tick*dt,dt)
            state = host.snapshot()
            require(abs(state['truthPosition_w[3]']-(-1+0.5*9.80665*0.02**2))<2e-5, 'Freefall must be Down-positive')
            require(abs(state['acceleration_frd[3]'])<1e-8, 'Freefall specific force must be zero')
            demand = [(i+1)/(2*n) for i in range(n)]
            host.set_inputs([1000+1000*u for u in demand])
            for tick in range(200,600):
                host.step(tick*dt,dt)
            state = host.snapshot()
            errors = [abs(state[f'rotorSpeed[{i+1}]']-1000*u*(1-math.exp(-0.04/0.03))) for i,u in enumerate(demand)]
            require(max(errors)<1.0, f'Independent P0 motor response failed: {errors}')
            for i in range(1,4):
                require(abs(state[f'position_ned[{i}]']-state[f'truthPosition_w[{i}]'])<1e-8, 'GNSS must already be NED')
                require(abs(state[f'velocity_ned[{i}]']-state[f'truthVelocity_w[{i}]'])<1e-8, 'Velocity converted twice')
            require(abs(state['altitude_m']+state['position_ned[3]'])<1e-8, 'Barometer altitude must be Up-positive')
            require(abs(state['climbRate_mps']+state['velocity_ned[3]'])<1e-8, 'Climb rate must be Up-positive')
            require(state['acceleration_frd[3]']<0, 'Thrust must give negative body Down specific force')
            metrics[preset] = {"channels":n,"max_speed_error_rad_s":max(errors),"step_s":dt}
        finally:
            host.close()
        # A fresh instance exercises touchdown/liftoff events inside the exported FMU.
        # Never overwrite a shared library that is still mapped by ctypes.
        host = Fmi2CS(fmu, work / "unpacked-ground", n)
        try:
            host.initialize([1000]*n, 5.501)
            checkpoints = {}
            takeoff_pwm = 1000+math.sqrt(1.3*1.5*9.80665/(n*1e-5))
            for tick in range(55000):
                if tick == 20000:
                    host.set_inputs([takeoff_pwm]*n)
                elif tick == 27000:
                    host.set_inputs([1000]*n)
                host.step(tick*dt,dt)
                if tick+1 in (15000,25000,55000):
                    checkpoints[(tick+1)*dt] = host.snapshot()
            for time in (1.5,5.5):
                state = checkpoints[time]
                require(abs(state['truthPosition_w[3]']-(-0.1+1.5*9.80665/12000))<1e-5,
                        f'Four-point FMU support/compression failed at {time}')
                require(abs(state['truthVelocity_w[3]'])<1e-5, f'FMU did not settle at {time}')
                require(abs(state['acceleration_frd[3]']+9.80665)<1e-3,
                        f'Grounded FMU accelerometer must report -g at {time}')
            airborne = checkpoints[2.5]
            require(airborne['truthPosition_w[3]'] < -0.15 and airborne['truthVelocity_w[3]'] < 0,
                    'FMU must release contact and rise during thrust')
            metrics[preset]['landing_gear'] = {
                'settled_altitude_m': -checkpoints[1.5]['truthPosition_w[3]'],
                'airborne_altitude_m': -airborne['truthPosition_w[3]'],
                'landed_altitude_m': -checkpoints[5.5]['truthPosition_w[3]'],
                'supported_acceleration_frd_z_mps2': checkpoints[1.5]['acceleration_frd[3]'],
            }
        finally:
            host.close()
    report = {"compiler":subprocess.check_output([omc,"--version"],text=True).strip(),
              "target":"OpenModelica FMI 2.0 Co-Simulation", "core_contract":"ned_frd_continuous_v1",
              "output_refresh":"Reapply held inputs before getReal (OpenModelica 1.26.1 contact-event cache workaround)",
              "metrics":metrics}
    (output / "report.json").write_text(json.dumps(report,indent=2)+"\n")
    print("PASS: quad/hexa FMU generation, motor/sensor outputs, settling, takeoff and landing")
    print(output / "report.json")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--omc",default="omc")
    parser.add_argument("--output",type=Path,default=ROOT / "build/export-validation")
    args = parser.parse_args()
    run(args.output.resolve(),args.omc)
