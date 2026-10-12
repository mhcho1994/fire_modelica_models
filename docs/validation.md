# Minimum core validation

Run from the library root with OpenModelica and Modelica 4.0.0 installed:

```sh
omc tools/simulate_landing_gear.mos
python3 tools/verify.py
FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p test_composer.py
python3 tools/probe_export.py
cargo clippy --locked --offline --manifest-path tools/fire-compose/Cargo.toml --all-targets -- -D warnings
```

`verify.py` runs every active physics test and all hover/standalone-IMU examples.
It checks analytical P0 speed response and saturation, local aerodynamic torque signs,
CG moment arms/channel mapping, replacement rotor coefficients, quaternion dynamics,
full inertia, freefall, hover, static mass/payload rebasing and invalid mass configurations.
`NedFrdContract` compares full ENU/FLU and NED/FRD motion at a nontrivial attitude,
with off-diagonal inertia, rotated/displaced IMU, magnetic field, GNSS antenna velocity
and Up-positive barometer altitude. `ContinuousSignals` checks values between former
sampling instants. Quad/hexa/octo/coaxial hover altitude is +1 m (NED z=-1 m).
Default local IMU z is +9.80665 m/s2; body FRD specific-force z is negative.

Landing-gear tests additionally check unilateral force (no tension), inclined surface
projection, Coulomb-capped dissipative friction, angular tip velocity, CG rebasing and
moment balance, zero-leg/freefall behavior, uneven touchdown, and four-foot quad/hexa
settling → takeoff → second landing. The static load is mg, compression is mg/(4k), and
the supported body-FRD accelerometer reads -g. The physics assertions run with native DASSL.
`Examples.QuadGroundCycle` is also available directly in OMEdit; its CLI script writes
`build/landing-gear/QuadGroundCycle_res.csv` for plotting.

`test_composer.py` checks schema-3 defaults, explicit rejection of schema 1/2 and deprecated
settings, nested-table selection, source/manifest ownership, frame metadata, omission of
archived sources, and native simulations of generated quad/hexa FastDyn boundaries.
Generated quad/hexa tests also exercise custom foot positions, ground height and contact
coefficients, grounded accelerometer outputs and takeoff. Invalid landing-gear shapes,
negative/nonfinite coefficients and nonboolean enable flags are rejected before emission.
Set FIRE_TEST_OMC=1 to include these generated-model simulations.
Mass metadata tests reject duplicate nonempty IDs across core, arms, payloads and
additional parts before emission, allow blank IDs, and verify that IDs remain in the
manifest without affecting generated Modelica. The generated assembled-payload test
checks mass and CG; `ChassisMassProperties` checks the full rotated inertia tensor.
Duplicate IDs are tested in the composer rather than `ChassisRejectedConfigurations`,
whose remaining negative cases cover physical validity in Modelica.

`probe_export.py` builds quad/hexa FMI 2.0 CS artifacts, initializes real FMU instances,
sets all channels, calls doStep/getReal and checks freefall, independent motor response,
FRD specific-force sign, NED position/velocity and Up-positive altitude/climb rate.
The 0.1 ms FMI step permits <1 rad/s motor-speed error versus the analytic solution;
native solver tests use tighter model-specific tolerances. A second instance of each FMU
checks unpowered settling, symmetric-thrust liftoff and landing after motor cutoff.
The probe currently targets Linux64.

**OpenModelica 1.26.1 FMI runtime workaround:** with contact event indicators,
`internalGetEventIndicators` evaluates ODE equations then clears `_need_update`, which
can leave algebraic sensor outputs one CS step behind the state-derived truth outputs.
The probe reapplies the unchanged held PWM inputs immediately before `getReal` to invalidate
that cache and refresh all outputs. This uses ordinary FMI calls and changes no input values
or continuous states. The report records this workaround; these results do not certify a
host that omits the refresh or uses the uncorrected runtime without it. The native DASSL
simulations do not require this workaround. No installed compiler/runtime files are modified.

Native and FMU reports are written under build/verification and build/export-validation.
They describe the current sources only. Historical ENU/FLU contact, sampling, fixed-wing
and rover results remain with the archived package and do not certify this profile.
These checks do not exercise Rumoca component RBC reuse/linking, FMI 3 packaging, actual
FastDyn firmware integration, or flight calibration. Aircraft coefficients are illustrative.

## P1 extension validation (2026-10-12)

`IdealEscVoltage`, `VoltagePropulsion` and `VoltageMultirotor` check command clamping,
per-channel PWM ranges, supply steps/zero supply, nonzero initial speed, the analytical
first-order trajectory, independent speed states, nominal P0/P1 equivalence and common
vehicle wrench/contact/sensor behavior. They pass with OpenModelica 1.26.1.
`Examples.QuadVoltageSag` also simulates successfully: its 16-to-12 V supply change
reduces speed from 606.42343 to 454.81757 rad/s, a ratio of 0.75.

All 19 composer tests pass with `FIRE_TEST_OMC=1`, including generated P1 quad/hexa
`plant` and `vio` profiles, independent gains/time constants/initial speeds,
nontrivial actuator mapping, invalid/cross-profile parameters, and verification-cache
invalidation when selecting P1. Rust clippy passes with warnings denied.

The complete native suite currently passes 24/27 simulations. The failures are
`ChassisMassProperties` (aggregate mass), `ContactWrenchCG` (shifted-CG contact moment)
and `MultirotorFrames` (shifted-CG hub moment). All three reproduce with the **unchanged
pre-P1 HEAD `fedff92`** in a separate package copy under the same compiler. They are
existing failures, not a claim that the entire P0 suite is green. The main verification
runner stops at these failures, before its separate invalid-chassis checks.
Artifacts for this run are in `build/p1-verification/`, including `native.log`,
`prechange-baseline.log` and `p1-report.json`; composer output is in `build/p1-composer-tests.log`.

Run the additional FMI 3 integration tests from the **FastDyn repository root**:

```sh
FASTDYN_TEST_RUMOCA_DIR=third_party/common/rumoca PYTHONPATH=src \
  python3 -m unittest discover -s tests/integration -p test_fire_propulsion_profiles.py
```

All four cases pass: P0 quad, nominal P1 quad, half-voltage P1 quad, and P1 hexa with
8 PWM channels mapped onto 6 rotors. Each case composes, compiles with Rumoca, packages
the source FMU, loads the packaged binary and its own metadata, initializes and calls
`fmi3DoStep` for 0.08 s with a 0.1 ms step. Checks cover the sole external input `pwm`,
array shapes, channel mapping, analytical rotor speed (absolute tolerance 1e-3 rad/s),
finite sensor outputs and the internal DC supply. Logs are in FastDyn's `out/virtualio-fmi3-tests.log`.
This test does not run flight-controller firmware or validate battery/current/shaft physics.

A persistent runnable P1 quad artifact can be built from FastDyn with:

```sh
python3 utils/build_fmi3_fmu.py --config configs/copter462_fire_p1.toml --skip-submodules
```

`configs/copter462_fire_p0.toml` selects P0 (renamed from `copter462_fire.toml`). Both profiles retain
identical external PWM and sensor field names; P1's additional voltage connection is internal.


## Shared VirtualIO update validation (2026-10-12)

The active adapter package is now `Adapters.VirtualIO`. Both `Adapters.VIO` and
`Adapters.FastDyn` are removed, along with the compatibility-only command redeclaration
and legacy wrapper test. The `vio` profile name and `fastdyn` CLI alias remain.

All 19 composer tests pass, including canonical/alias equivalence for P0/P1, shared
adapter metadata, supply binding and regeneration of an existing staged library.
The staging test seeds obsolete VIO/FastDyn packages and verifies that regeneration
removes both. Source manifests also exclude these removed classes.
`AdapterChannels`, `ContinuousSignals`, `NedFrdContract` and `IdealEscVoltage` use the
canonical VirtualIO classes. The native suite passes 24/27 with the same three baseline
failures above; removal of the obsolete legacy test accounts for the reduced case count.
Current logs: `build/virtualio-verification/native.log` and
`build/virtualio-composer-tests.log`. Earlier P1 logs retain baseline evidence.

All four FMI3 cases pass with VirtualIO, including out-of-range PWM clamping, channel
mapping, analytical speed, sensor outputs, metadata and P1 internal supply. The standalone
`VirtualIO.MultirotorFmu` also passes the quad/hexa FMI2 export/runtime probe, including
motor/sensor outputs, settling, takeoff and landing. Its report is in
`build/virtualio-export-validation/report.json`. Rust clippy passes with warnings denied.
The 13 FastDyn configuration/cache unit tests passed during the preceding common-adapter
update; the VirtualIO rename preserves those configuration names and paths.

Actual flight-controller execution remains unverified. The FastDyn C FMU host still
handles four PWM channels, whereas the Python FMI3 probe is array-aware and tests the
eight-channel/six-rotor hexa artifact.

Both persistent FastDyn configs were rebuilt successfully after the rename:
`configs/copter462_fire_p0.toml` -> `out/fmi3/ConfiguredQuad/ConfiguredQuad.fmu` and
`configs/copter462_fire_p1.toml` -> `out/fmi3/ConfiguredQuadP1/ConfiguredQuadP1.fmu`.
Their staged libraries contain only `Adapters/VirtualIO`. Build logs are
`out/virtualio-p0-build.log` and `out/virtualio-p1-build.log` in FastDyn.
