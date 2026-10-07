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
