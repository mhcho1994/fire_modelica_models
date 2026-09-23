#!/usr/bin/env python3
"""Build and exercise the actual OpenModelica FMI 2.0 CS artifact, using stdlib only."""

from __future__ import annotations

import argparse
import csv
import ctypes as ct
import hashlib
import json
import math
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL = "FIRE_Modelica.Tests.ExportOcto"
PERIODS = (0.0025, 0.02, 0.2, 0.02)
GROUPS = (
    [f"{name}[{i}]" for name in ("acceleration_frd", "gyro_frd") for i in range(1, 4)],
    [f"magneticField_frd[{i}]" for i in range(1, 4)],
    [f"{name}[{i}]" for name in ("position_ned", "velocity_ned") for i in range(1, 4)],
    ["pressure_Pa", "temperature_K", "altitude_m", "climbRate_mps"],
)
TIMESTAMPS = [f"sampleTimes[{i}]" for i in range(1, 5)]
ROTOR_SPEEDS = [f"rotorSpeed[{i}]" for i in range(1, 9)]
INPUTS = [f"pwm_us[{i}]" for i in range(1, 9)]
DEMANDS = [f"sampledDemand[{i}]" for i in range(1, 9)]
NORMALS = [f"normalForce[{i}]" for i in range(1, 5)]
GAPS = [f"gap[{i}]" for i in range(1, 5)]
CONTACTS = [f"contact[{i}]" for i in range(1, 5)]
OBSERVABLES = list(dict.fromkeys(
    ROTOR_SPEEDS + DEMANDS + NORMALS + GAPS + TIMESTAMPS
    + ["commandSampleTime", "truthPosition_w[3]", "truthVelocity_w[3]"]
    + [name for group in GROUPS for name in group]
))


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def source_identity(package: Path) -> dict:
    root = package.resolve().parent
    digest = hashlib.sha256()
    count = 0
    for path in sorted(root.rglob("*.mo")):
        relative = path.relative_to(root)
        if relative.parts[0] in {"build", ".git"}:
            continue
        digest.update(relative.as_posix().encode() + b"\0" + path.read_bytes() + b"\0")
        count += 1
    git = shutil.which("git")
    revision = None
    if git:
        result = subprocess.run([git, "-C", str(root), "rev-parse", "HEAD"],
                                capture_output=True, text=True, check=False)
        if result.returncode == 0:
            revision = result.stdout.strip()
    return {"package": str(package.resolve()), "git_revision": revision,
            "modelica_source_sha256": digest.hexdigest(), "modelica_file_count": count,
            "digest_scope": "sorted relative .mo paths and file bytes; excludes build/ and .git/"}


def build_fmu(output: Path, package: Path) -> tuple[Path, str]:
    require(package.is_file(), f"Modelica package does not exist: {package}")
    compiler = shutil.which("omc")
    require(compiler is not None, "OpenModelica omc was not found on PATH")
    version = subprocess.check_output([compiler, "--version"], text=True).strip()
    script = output / "export.mos"
    # Modelica and JSON use the same escaping for this ordinary quoted path.
    script.write_text(
        'loadModel(Modelica,{"4.0.0"});\n'
        f"loadFile({json.dumps(str(package.resolve()))});\n"
        "getErrorString();\n"
        f'buildModelFMU({MODEL}, version="2.0", fmuType="cs", '
        'fileNamePrefix="ExportOcto", platforms={"static"});\n'
        "getErrorString();\n"
    )
    completed = subprocess.run([compiler, str(script)], cwd=output,
                               capture_output=True, text=True, timeout=300)
    log = completed.stdout + completed.stderr
    (output / "build.log").write_text(log)
    require(completed.returncode == 0 and "Error:" not in log,
            f"OpenModelica export failed; see {output / 'build.log'}")
    fmu = output / "ExportOcto.fmu"
    require(fmu.is_file() and f'"{fmu}"' in log,
            "No new FMU was returned by buildModelFMU; refusing a potentially stale artifact")
    return fmu, version


class Fmi2CS:
    """Small FMI 2.0 Co-Simulation host; no fallback to native Modelica simulation."""

    def __init__(self, fmu: Path, directory: Path):
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
        for name in INPUTS:
            require(name in self.variables and self.variables[name].attrib.get("causality") == "input",
                    f"Missing real PWM input {name}")
        for name in ROTOR_SPEEDS + TIMESTAMPS + CONTACTS + NORMALS + GAPS:
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
        self._bind("fmi2GetBoolean", [ct.c_void_p, ct.POINTER(ct.c_uint), ct.c_size_t, ct.POINTER(ct.c_int)])
        self._bind("fmi2DoStep", [ct.c_void_p, ct.c_double, ct.c_double, ct.c_int])
        self.component = self.fmi2Instantiate(
            b"fire_export_probe", 1, description.attrib["guid"].encode(),
            ((directory / "resources").resolve().as_uri() + "/").encode(),
            ct.byref(self.callbacks), 0, 0)
        require(bool(self.component), "fmi2Instantiate failed")

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
        self._check(self.fmi2SetReal(self.component, self._references(INPUTS), len(INPUTS),
                                   (ct.c_double * len(values))(*values)), "fmi2SetReal")

    def initialize(self, values: list[float], stop: float) -> None:
        self._check(self.fmi2SetupExperiment(self.component, 1, 1e-8, 0, 1, stop), "setup")
        self.set_inputs(values)
        self._check(self.fmi2EnterInitializationMode(self.component), "enter initialization")
        self._check(self.fmi2ExitInitializationMode(self.component), "exit initialization")

    def snapshot(self) -> dict[str, float]:
        values = (ct.c_double * len(OBSERVABLES))()
        self._check(self.fmi2GetReal(self.component, self._references(OBSERVABLES), len(values), values), "get real")
        require(all(math.isfinite(value) for value in values), "Nonfinite exported real value")
        result = dict(zip(OBSERVABLES, values))
        booleans = (ct.c_int * len(CONTACTS))()
        self._check(self.fmi2GetBoolean(self.component, self._references(CONTACTS), len(booleans), booleans), "get boolean")
        result.update(zip(CONTACTS, booleans))
        return result

    def step(self, time: float, step: float) -> None:
        self._check(self.fmi2DoStep(self.component, time, step, 1), "fmi2DoStep")

    def close(self) -> None:
        if self.component:
            self.fmi2Terminate(self.component)
            self.fmi2FreeInstance(self.component)
            self.component = None


class SampleAudit:
    def __init__(self):
        self.previous = None
        self.updates = [0] * 4
        self.maximum_age = [0.0] * 4

    def observe(self, time: float, state: dict, step: float) -> None:
        for index, (period, group, key) in enumerate(zip(PERIODS, GROUPS, TIMESTAMPS)):
            stamp = state[key]
            require(abs(stamp / period - round(stamp / period)) < 1e-6,
                    f"Sensor timestamp is off its acquisition grid: {key}={stamp}")
            age = time - stamp
            require(-step - 1e-9 <= age <= period + step + 1e-9,
                    f"Sensor timestamp stopped advancing or lies in the future: {key}={stamp}, t={time}")
            self.maximum_age[index] = max(self.maximum_age[index], age)
            if self.previous is not None:
                require(stamp >= self.previous[key] - 1e-10, "Sensor timestamp moved backwards")
                if abs(stamp - self.previous[key]) < 1e-10:
                    require(all(abs(state[name] - self.previous[name]) < 1e-9 for name in group),
                            f"Sensor outputs changed between acquisition events: {key}, t={time}")
                else:
                    self.updates[index] += 1
        self.previous = state


def run_case(fmu: Path, output: Path, case: str, step: float) -> dict:
    stop = 0.05 if case == "channels" else 2.5
    ticks = round(stop / step)
    require(abs(ticks * step - stop) < 1e-10, "Step must divide both scenario durations")
    require(abs(round(2 / step) * step - 2) < 1e-10, "Step must divide the takeoff-command time")
    initial = [1100 + 50 * index for index in range(8)] if case == "channels" else [1000] * 8
    instance = Fmi2CS(fmu, output / f"unpacked_{case}")
    audit = SampleAudit()
    rows = []
    touchdown = None
    liftoff = None
    support = []
    minimum_gap = 0.0
    try:
        instance.initialize(initial, stop)
        for tick in range(ticks):
            time = tick * step
            if case == "contact" and tick == round(2 / step):
                instance.set_inputs([1700] * 8)
            instance.step(time, step)
            now = (tick + 1) * step
            state = instance.snapshot()
            audit.observe(now, state, step)
            require(min(state[name] for name in NORMALS) >= -1e-9, "Ground contact became tensile")
            for gap, normal in zip(GAPS, NORMALS):
                require(state[gap] <= 1e-8 or abs(state[normal]) < 1e-8,
                        "Positive gap produced a ghost contact force")
            contact_count = sum(state[name] for name in CONTACTS)
            if contact_count and touchdown is None:
                touchdown = now
            if now > 2 and touchdown is not None and contact_count == 0 and state["truthVelocity_w[3]"] > 0:
                liftoff = now if liftoff is None else liftoff
            if 1.8 <= now < 2:
                support.append(state)
            minimum_gap = min(minimum_gap, *(state[name] for name in GAPS))
            rows.append({"time": now, **state})
        last = rows[-1]
        metrics = {"duration_s": stop, "steps": ticks, "sensor_updates": audit.updates,
                   "maximum_sample_age_s": audit.maximum_age, "fmi_log": instance.log}
        if case == "channels":
            expected = [(pwm - 1000) * (1 - math.exp(-stop / 0.03)) for pwm in initial]
            actual = [last[name] for name in ROTOR_SPEEDS]
            error = max(abs(a - e) / max(abs(e), 1) for a, e in zip(actual, expected))
            require(error < 0.015, f"Eight-channel motor response mismatch: max relative error {error}")
            require(all(abs(last[name] - (pwm - 1000) / 1000) < 1e-10
                        for name, pwm in zip(DEMANDS, initial)), "PWM channels were mis-mapped or truncated")
            metrics.update(pwm_us=initial, expected_rotor_speed=expected, actual_rotor_speed=actual,
                           maximum_relative_motor_error=error)
        else:
            gravity, mass, stiffness = 9.80665, 1.5, 1500.0
            analytic_touchdown = math.sqrt(2 * 0.35 / gravity)
            require(touchdown is not None and abs(touchdown - analytic_touchdown) < 0.004,
                    f"Touchdown timing mismatch: {touchdown}, expected {analytic_touchdown}")
            require(support and all(sum(item[name] for name in CONTACTS) == 4 for item in support),
                    "Vehicle did not settle onto all four legs")
            normal = sum(sum(item[name] for name in NORMALS) for item in support) / len(support)
            support_z = sum(item["truthPosition_w[3]"] for item in support) / len(support)
            expected_z = 0.15 - mass * gravity / (4 * stiffness)
            require(abs(normal - mass * gravity) < 0.03, "Supported normal force does not balance weight")
            require(abs(support_z - expected_z) < 1e-5, "Static spring compression is incorrect")
            require(max(abs(item["truthVelocity_w[3]"]) for item in support) < 1e-3, "Supported vehicle is not stationary")
            require(max(abs(item["acceleration_frd[3]"] + gravity) for item in support) < 0.03,
                    "Supported specific force did not cross the FLU-to-FRD boundary correctly")
            require(liftoff is not None and last["truthPosition_w[3]"] > 0.3 and
                    sum(last[name] for name in CONTACTS) == 0, "Commanded liftoff failed")
            require(all(count > 0 for count in audit.updates), "Not all four sensor clocks advanced")
            metrics.update(touchdown_s=touchdown, analytical_touchdown_s=analytic_touchdown,
                           liftoff_s=liftoff, supported_normal_force_N=normal,
                           supported_height_m=support_z, expected_supported_height_m=expected_z,
                           maximum_penetration_m=-minimum_gap, final_height_m=last["truthPosition_w[3]"])
        return metrics
    finally:
        instance.close()
        if rows:
            with (output / f"{case}.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, default=ROOT / "package.mo")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "build" / "export")
    parser.add_argument("--fmu", type=Path, help="Reuse an existing ExportOcto FMU instead of building")
    parser.add_argument("--step", type=float, default=0.00025, help="FMI communication step in seconds")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.is_relative_to(ROOT) and not output.is_relative_to(ROOT / "build"):
        parser.error("Output directory must be under this repository's build/ or outside the repository")
    output.mkdir(parents=True, exist_ok=True)
    metrics = {"model": MODEL, "backend": "OpenModelica FMI 2.0 Co-Simulation",
               "modelica_library": "4.0.0",
               "communication_step_s": args.step if math.isfinite(args.step) else None,
               "rumoca": "not executed; no local binary available"}
    try:
        require(math.isfinite(args.step) and args.step > 0, "Communication step must be finite and positive")
        if args.fmu:
            fmu, version = args.fmu.resolve(), "existing artifact; see modelDescription.xml"
            metrics["source"] = {"association": "unknown for reused artifact"}
        else:
            metrics["source"] = source_identity(args.package)
            fmu, version = build_fmu(output, args.package)
        metrics.update(fmu=str(fmu), compiler=version,
                       fmu_sha256=hashlib.sha256(fmu.read_bytes()).hexdigest())
        for case in ("channels", "contact"):
            metrics[case] = run_case(fmu, output, case, args.step)
        metrics["status"] = "passed"
    except (AssertionError, OSError, subprocess.SubprocessError, KeyError, ValueError) as error:
        metrics.update(status="failed", error=str(error))
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")
    print(json.dumps(metrics, indent=2, allow_nan=False))
    return 0 if metrics["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
