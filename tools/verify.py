#!/usr/bin/env python3
"""Run native OpenModelica physics assertions and numerical integration regressions.

Requires omc and Modelica 4.0.0. Artifacts and a machine-readable report go to
build/verification. This is not a Rumoca or FMU test; use probe_export.py separately.
"""
import argparse
import csv
import json
import math
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def quoted(value):
    return json.dumps(str(value))


def experiment(path):
    text = path.read_text()
    def number(name, default):
        match = re.search(rf"\b{name}\s*=\s*([0-9.eE+-]+)", text)
        return float(match.group(1)) if match else default
    stop = number("StopTime", 1)
    return stop, number("Tolerance", 1e-8), max(50, math.ceil(stop/number("Interval", 0.002)))


def final_row(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise AssertionError(f"Empty result: {path}")
    return rows[-1]


def run(omc, out):
    out.mkdir(parents=True, exist_ok=True)
    cases = []
    for path in sorted((ROOT/"Tests").glob("*.mo")):
        if path.stem in {"package", "ExportOcto", "ChassisRejectedConfigurations"}:
            continue
        cases.append((f"fire_modelica_models.Tests.{path.stem}", *experiment(path)))
    for name in ("QuadHover", "QuadImuOnly", "QuadImuResponse", "HexaHover", "OctoHover", "CoaxialHover", "QuadDrop", "FixedPayload",
                 "SkywalkerX8SITLScenario", "R1RoverSITLScenario"):
        stop = 3 if name in {"QuadDrop", "FixedPayload"} else 1
        cases.append((f"fire_modelica_models.Examples.{name}", stop, 1e-8, int(stop/0.002)))
    script = ['loadModel(Modelica,{"4.0.0"});', f'loadFile({quoted(ROOT/"package.mo")});',
              'getErrorString();']
    for name, stop, tol, intervals in cases:
        short = name.split(".")[-1]
        script.append(f'simulate({name},stopTime={stop},tolerance={tol},numberOfIntervals={intervals},'
                      f'outputFormat="csv",fileNamePrefix="{short}");')
        script.append('getErrorString();')
    script += [f'loadFile({quoted(ROOT/"compat/FIRE_Modelica_Update/package.mo")});',
               'checkModel(FIRE_Modelica_Update.Examples.SkywalkerX8SITLScenario);',
               'checkModel(FIRE_Modelica_Update.Examples.R1RoverSITLScenario);', 'getErrorString();']
    script_path = out/"verify.mos"
    script_path.write_text("\n".join(script)+"\n")
    # Avoid accepting stale results if a later compiler run fails.
    for name, *_ in cases:
        (out/(name.split(".")[-1]+"_res.csv")).unlink(missing_ok=True)
    result = subprocess.run([omc, str(script_path)], cwd=out, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300)
    (out/"native.log").write_text(result.stdout)
    successes = result.stdout.count("The simulation finished successfully.")
    if result.returncode or "Error:" in result.stdout or successes != len(cases):
        raise AssertionError(f"Native run: {successes}/{len(cases)} passed; see {out/'native.log'}")
    metrics = {}
    for name in ("QuadHover", "HexaHover", "OctoHover", "CoaxialHover"):
        row = final_row(out/(name+"_res.csv"))
        altitude = float(row["vehicle.body.p_w[3]"])
        acc = float(row["vehicle.measurements.acceleration[3]"])
        assert abs(altitude-1)<1e-7 and abs(acc-9.80665)<1e-7, name
        metrics[name] = {"altitude_m": altitude, "specific_force_up_mps2": acc}
    row = final_row(out/"QuadDrop_res.csv")
    expected_height = 0.15-1.5*9.80665/(4*1500)
    assert abs(float(row["vehicle.body.p_w[3]"])-expected_height)<1e-7
    assert abs(float(row["vehicle.body.v_w[3]"]))<1e-7
    assert all(float(row[f"vehicle.landingGear.contact[{i}]"])==1 for i in range(1,5))
    metrics["QuadDrop"] = {"settled_height_m": float(row["vehicle.body.p_w[3]"]),
                            "predicted_height_m": expected_height}
    with (out/"QuadDrop_res.csv").open() as stream:
        drop_rows = list(csv.DictReader(stream))
    switches = [(a,b) for a,b in zip(drop_rows,drop_rows[1:])
                if a["vehicle.landingGear.contact[1]"] != b["vehicle.landingGear.contact[1]"]]
    assert switches, "Drop must actually trigger a contact event"
    touchdown = float(switches[0][1]["time"])
    predicted_touchdown = math.sqrt(2*(0.5-0.15)/9.80665)
    assert abs(touchdown-predicted_touchdown)<1e-6
    jumps = []
    for before, after in switches:
        assert abs(float(before["time"])-float(after["time"]))<1e-12
        jumps.append(abs(float(after["vehicle.body.v_w[3]"])-float(before["vehicle.body.v_w[3]"])))
    assert max(jumps)<1e-8, "Contact must not reset continuous velocity"
    metrics["QuadDrop"].update(touchdown_s=touchdown, predicted_touchdown_s=predicted_touchdown,
                               contact_switches=len(switches), max_velocity_jump_mps=max(jumps))
    rejected = {
        "DuplicateIds": "Duplicate physical mass componentId",
        "ImproperRotation": "right-handed rotation",
        "IndefiniteInertia": "physical mass distribution",
        "NonphysicalPrincipalMoments": "physical mass distribution",
        "MasslessInertia": "zero-mass part cannot carry nonzero mass inertia",
    }
    for name, diagnostic in rejected.items():
        mos = out/("reject_"+name+".mos")
        mos.write_text('loadModel(Modelica,{"4.0.0"});\n'
                       f'loadFile({quoted(ROOT/"package.mo")});\n'
                       f'simulate(fire_modelica_models.Tests.ChassisRejectedConfigurations.{name},'
                       f'stopTime=0.01,fileNamePrefix="reject_{name}");\ngetErrorString();\n')
        proc = subprocess.run([omc, str(mos)], cwd=out, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        (out/("reject_"+name+".log")).write_text(proc.stdout)
        if diagnostic not in proc.stdout or "The simulation finished successfully." in proc.stdout:
            raise AssertionError(f"Expected invalid configuration rejection: {name}")
    version = subprocess.check_output([omc,"--version"], text=True).strip()
    report = {"compiler": version, "target": "OpenModelica native DASSL",
              "passed_models": [c[0] for c in cases], "expected_rejections": list(rejected),
              "legacy_namespace_checks": 2, "metrics": metrics,
              "known_warnings": "Original legacy sensors have underspecified initial discrete buffers"}
    (out/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(f"PASS: {len(cases)} simulations, {len(rejected)} invalid configurations rejected, 2 namespace checks")
    print(out/"report.json")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--omc", default="omc")
    parser.add_argument("--output", type=Path, default=ROOT/"build/verification")
    args = parser.parse_args()
    run(args.omc, args.output.resolve())
