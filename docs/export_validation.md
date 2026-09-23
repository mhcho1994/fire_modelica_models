# Eventful FMU boundary validation

The reusable `Adapters.FastDyn.MultirotorFmu` boundary exports arbitrary
geometry-sized PWM inputs and rotor-speed outputs, flat sensor measurements,
acquisition timestamps, and separate continuous ENU/FLU truth. It composes the
native `MultirotorValueAdapter`; it does not change its dynamics or sensor clocks.
`Tests.ExportOcto` selects eight rotors, starts at 0.5 m, and adds contact and
sampled-demand diagnostics used only for validation.

Run from the repository root:

```sh
python3 tools/probe_export.py
```

The tool requires OpenModelica `omc`, Python 3.10+, and a Linux x86-64 host. It
pins Modelica 4.0.0, invokes `buildModelFMU` for FMI 2.0 Co-Simulation, extracts
the produced artifact, and calls the actual shared-library FMI API through
Python's standard-library `ctypes`. No FMPy installation or network access is
needed. No special compiler/debug flags are used. Artifacts go into ignored
`build/export/`: the FMU, compiler log, scenario CSV files, and `metrics.json`.
Fresh-build metrics record the FMU SHA-256, FIRE Git revision, and a digest of
the package's `.mo` filenames and contents. Reused artifacts retain their own
FMU hash but are not attributed to the current source tree.
An assertion failure or a failed FMI operation returns a nonzero exit code.

An existing artifact can be retested with
`python3 tools/probe_export.py --fmu build/export/ExportOcto.fmu`.
`--output-dir` must be under this repository's `build/` directory or outside the
repository. The default communication step is 0.00025 s; supported custom steps
must divide the scenario durations and the 2 s takeoff command time.

The executed checks are:

- Eight distinct PWM inputs remain eight exported inputs and map to eight
  distinct rotor responses; each speed agrees with the first-order analytical
  response within the declared 1.5% bound.
- IMU, magnetometer, GNSS, and barometer acquisition times stay on their separate
  grids, advance throughout the run, and hold their measurements between updates.
  At an exact communication/event boundary, the previous sample may remain
  visible until the following communication step; the audit allows one step of
  event-boundary tolerance and verifies held values explicitly.
- A zero-command drop produces touchdown, four-leg support, nonnegative normal
  force, and no contact force across a positive gap. Static normal force balances
  weight, spring compression matches equilibrium, and supported accelerometer
  output has the correct FRD sign.
- A later command produces liftoff and rising altitude without disabling any
  contact or sampling events.

OpenModelica 1.26.7 (`1.26.7~1-g2b913cc`) passed the complete artifact test with
Modelica 4.0.0 and the default communication step:

| Measured quantity | Result |
|---|---:|
| Maximum relative motor-speed error | 0.1621% |
| Touchdown | 0.2675 s |
| Analytical freefall touchdown | 0.26717061 s |
| Settled total normal force | 14.709975 N |
| Settled CG height | 0.1475483375 m |
| Liftoff after the 2 s command | 2.0505 s |
| CG height at 2.5 s | 1.73863 m |
| FMI communication steps exercised | 10,200 |

Two source formulations matter for this verified compiler version. New sampled
sensors use independent `when` equations, preserving the same acquisition/hold
behavior as their algorithm form. The rotation utility provides scalar elements
to the rigid-body equations and builds matrices from that same scalar formula;
this avoids OpenModelica FMI residual and array-derivative failures. Nonunit,
zero, and near-zero quaternion values retain the previous `max(norm, eps)`
normalization behavior. The drag and tangential-friction norms have an exact
zero-speed arithmetic branch so generated derivatives do not evaluate `0/0`
at rest. This branch neither smooths nor removes ground-contact switching.

This is an **OpenModelica FMU** result. Rumoca export/runtime and the real
FastDyn host have not been exercised by this tool. The local Rumoca checkout
had no executable on PATH or existing `target/` build. A separate offline,
locked CLI build attempt stopped before compilation because extracting cached
`diffsol` sources required writing into the read-only `~/.cargo/registry/src`
directory (OS error 30). No Rumoca source or lockfile was changed; see
[validation.md](validation.md) for that command and its result. FMI 3, Model Exchange, other platforms, and
electrical/EMI profiles require their own execution checks.
