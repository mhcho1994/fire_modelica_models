# Minimum continuous multicopter architecture

This implementation follows MULTICOPTER_INTERFACES v0.8 when the older port-contract
summary is ambiguous about frames. Active core contract: `ned_frd_continuous_v1`.
Contract IDs describe physical boundaries; profiles P0/P1 and R0 describe approximations.

## Composition and ownership

```text
P0: PwmInput -> ESC.PassThrough -> Motors.FirstOrderSpeed
                                      -> QuadraticBlade -> MultirotorWithSensors
P1: PwmInput -> ESC.AverageVoltage -> Motors.FirstOrderVoltageToSpeed
                                      -> QuadraticBlade -> MultirotorWithSensors
```

ESCs, motors and aerodynamic rotors are independent instances owned by the composer.
Only motors carry speed states. `MultirotorPlant` accepts rotor-local hub force and
moment arrays, returns `rotorAirVelocity_r` and `rotorOmegaBody_r`, and knows no command
or voltage model. `MultirotorWithSensors` is shared by P0/P1. The blade returns local
hub force/moment; P2 adds the axial inertia reaction before the vehicle, and only the vehicle adds `cross(rotorPosition_b, rotorForce_b)`.
Gravity is applied only by the rigid body.
External wrench inputs exclude gravity and are expressed about the combined CG.
The four landing contacts add their forces and CG moments to the same wrench sum.
Chassis mass assembly and massless compliant legs add no motion states. Aggregate and constituent
mass budgets are mutually exclusive; physical parts must be counted once.

`MassProperties` and its derived part records carry only `mass`, `r_C`, `R_Cj` and
`inertia`. Part `componentId` strings belong to composer configuration and the manifest,
not the numerical function interface. The composer rejects duplicate nonempty IDs in
assembled budgets before emission; blank IDs leave bookkeeping to the caller. Direct
Modelica constructors no longer accept `componentId`, so applications bypassing the
composer must check part identity themselves. `combineMassProperties` retains the
physical validity checks, rotations and parallel-axis calculation.

In P0/P1, the differential states are rotor speeds and rigid-body position, body velocity,
Hamilton quaternion and body angular velocity. Full inertia and quaternion dynamics
remain part of the minimum rigid-body contract. Sensors add no states. Fixed bias
parameters default to zero. Four-point spring/damper contact is enabled by default;
`landingGear(enabled=false)` removes it structurally. No body drag, sample/hold, sensor response filter,
noise, gyro rotor reaction, inflow or coaxial interference is active. P0/P1 have no power accounting.
`airVelocity_r` and `omegaBody_r` retain the R0 boundary but do not change R0 forces.

## Implemented ports

Physical and sensor values are continuously evaluated. Contact adds Boolean mode outputs
and state events, without introducing sampled acquisition. Scalar shape is `1`.
Names are actual Modelica names; the canonical `speedDemand` maps explicitly to `demand`.
The table documents implemented semantics; the composer does not yet validate arbitrary
component semantic contracts or preserve this metadata through Rumoca RBC.

| Boundary / model | Input (shape; SI unit) | Output (shape; SI unit) | Frame / reference |
|---|---|---|---|
| `speed_command_v1` / FirstOrderSpeed | demand (1; 1), normalized target speed, clamped [0,1] | omega (1; rad/s), nonnegative | shaft magnitude |
| R0 / QuadraticBlade | omega (1; rad/s), airVelocity_r (3; m/s), omegaBody_r (3; rad/s) | force_r (3; N), moment_r (3; N.m) | rotor hub; moment opposes spinSign |
| `rigid_body_wrench_v1` / RigidBody6DOF | force_b (3; N), moment_b (3; N.m), gravity_w (3; m/s2) | p_w (3; m), v_w/v_b (3; m/s), a_w/specificForce_b (3; m/s2), omega_b (3; rad/s), alpha_b (3; rad/s2), q_wb (4; 1), R_wb (3x3; 1) | NED world / FRD body at CG |
| LandingGearAssembly | p_w (3; m), v_b (3; m/s), omega_b (3; rad/s), R_wb (3x3), terrainPoint_w (3; m), terrainNormal_w (3), terrainVelocity_w (3; m/s) | force_b (3; N), moment_b (3; N.m), contact (nLegs; Boolean), gap (nLegs; m), normalForce (nLegs; N) | FRD wrench about combined CG; NED plane/kinematics |
| `imu_measurement_v1` / IMU.Sensor | R_wb (3x3; 1), a_w/gravity_w (3; m/s2), omega_b (3; rad/s), alpha_b (3; rad/s2) | acceleration (3; m/s2), gyro (3; rad/s) | sensor local at rigid mount |
| `mag_measurement_v1` / Magnetometer.Sensor | R_wb (3x3; 1), magneticField_w (3; T) | magneticField (3; T) | sensor local |
| `gnss_measurement_v1` / GNSS.Sensor | position_w (3; m), velocity_w (3; m/s) | position (3; m), velocity (3; m/s) | NED, antenna |
| `barometer_measurement_v1` / Barometer.Sensor | ambientPressure (1; Pa), ambientTemperature (1; K), altitude_w (1; m), climbRate_w (1; m/s) | pressure, temperature, altitude, climbRate (matching units) | scalar; altitude/climb rate Up-positive |

P0 motor parameters are `tau>0`, `omegaMax>0`, `0<=omega_start<=omegaMax`.
Rotor `spinSign` is +1/-1 about local +z. `moment_r` is the signed aerodynamic hub
moment, not a shaft `loadTorque`. P0 advertises no electrical or shaft-load port.
`mass`, `r_C`, `R_Cj`, `inertia` and geometry/mount arrays are static parameters.
IMU includes tangential and centripetal lever-arm acceleration. SensorSuite computes
GNSS/barometer mount velocity including `R_wb*cross(omega_b,r_b)`.
Atmosphere is constant; geometric barometer altitude is independent of supplied pressure.

## Landing gear and ground ownership

The active `Chassis.LandingGear` and `Contact` packages reuse the archived compliant
point-contact implementation. `LandingGear.Parameters` provides four chassis-relative
FRD tip positions, a horizontal plane at NED `groundZ`, and per-foot contact coefficients.
`MultirotorPlant` subtracts the combined CG from every tip and connects the body pose,
velocity and angular velocity to one `LandingGearAssembly`. Disabled gear has zero legs.
The plane is stationary and has outward normal `{0,0,-1}`; general terrain remains archived.

`ContactMode` activates at `gap<=0`. `UnilateralSpringDamper` applies a nonnegative
normal force; `TangentialFriction` applies dissipative viscous friction capped by `mu*Fn`.
No leg carries its own mass, compression state, joint or velocity reset. Count physical
leg mass in the chosen chassis mass budget, never by adding it again in the contact model.
The four-foot layout is independent of rotor count. Contact moments use `cross(rLeg_b,force_b)`
about the combined CG. External force/moment inputs remain independent and exclude gravity.

The default contact parameters are k=3000 N/m, c=150 N·s/m per foot, tangential damping
25 N·s/m and friction coefficient 0.6. Default feet are at ±0.17 m in X/Y and +0.10 m
in chassis Z. A horizontal symmetric 1.5 kg vehicle settles with 1.226 mm penetration.
Standalone initial CG position remains NED `{0,0,-1}`; the bundled quad/hexa TOML files
explicitly start at `{0,0,-0.12}`. Custom CG and attitude require explicit initial placement.
Contact transitions remain ordinary Modelica events, while sensors remain continuous.

## NED/FRD migration

For the same physical origin and pose:

```text
W = [0,1,0; 1,0,0; 0,0,-1]   # old ENU -> NED components
B = diag(1,-1,-1)             # old FLU -> FRD components
world_new = W * world_old
body_new = B * body_old
R_wb_new = W * R_wb_old * transpose(B)
I_new = B * I_old * transpose(B)
r_C_new = B * r_C_old
R_br_new = B * R_br_old
R_bs_new = B * R_bs_old
R_Cj_new = B * R_Cj_old        # when inertia is still in the original local j axes
```

Transform world biases/wind/magnetic fields as well; keep sensor-local biases and
spin signs unchanged. Recompute quaternion from the transformed attitude, rather than
flipping one component. Bundled TOML presets preserve old physical mounts, channel
indices, heading and height. Each explicit schema-3 geometry value is already FRD.
Standalone RigidBody6DOF and standalone sensors use identity attitude/mount defaults;
vehicle defaults retain the old East-facing pose and sensor local axes.

`Adapters.VirtualIO.SensorValues` applies sensor mount rotation to IMU/mag;
GNSS passes through as NED. `Exports.FMU.PartialCopterInterface` converts Tesla to Gauss,
Kelvin to Celsius and NED to approximate geodetic coordinates, and computes heading
as atan2(R_wb[2,1],R_wb[1,1]). `Exports.FMU.CopterInterface` adds the common PWM adapter.
The composer owns the constant atmospheric supply through explicit plant modifiers
`pressure=101325` and `temperature=288.15`. Export classes contain no environment source.
The composer PWM range remains 1100–1900 us, configurable through `[interface]`;
independent `PwmInput` defaults to 1000–2000 us.

Composer schema 3 and manifest version 2 explicitly record frames, core contract,
P0 or P1 with R0, continuous acquisition, no drag, and the selected contact profile.
`[landing_gear]` resolves to `LandingGear.Parameters`; the manifest sets
`physics.contact="four_point_spring_damper"` and `event_requirements.ground_contact=true`
when enabled, otherwise `"none"` and `false`. Archived files are excluded from
both staging and source hashing. Existing generated artifacts must be regenerated.

## Independent propulsion composition

The names describe behavior, while P0/P1 remain manifest labels for supported combinations:

| Selection | P0 | P1 |
|---|---|---|
| `models.esc` | `pass_through` | `average_voltage` |
| `models.motor` | `first_order_speed` | `first_order_voltage_to_speed` |
| `models.rotor` | `quadratic_blade` | `quadratic_blade` |

`Physical.Electrical.Drives.ESC.PassThrough` forwards [0,1] `escCommand` as
`normalizedCommand` with no state. `ESC.AverageVoltage` computes
`equivalentVoltage=clamp(escCommand,0,1)*dcVoltage`, with `dcVoltage>=0`.
It has no switching, delay, current or power port.
`Systems.Propulsion.Motors.FirstOrderSpeed` follows `omegaMax*demand` with `tau`.
`Motors.FirstOrderVoltageToSpeed` follows `Kv*equivalentVoltage` with `tauOmega`.
Both own exactly one speed state per rotor. P1 has no extra speed cap or load feedback;
zero voltage gives exponential coast-down. `Kv` uses (rad/s)/V, not rpm/V.

`[motor]` contains response parameters; `[rotor]` contains `kT/kQ`. The composer
maps actuator channels to ESCs, ESC outputs to motors, speeds to blades, and hub
wrenches to the common vehicle. The vehicle feeds hub kinematics back to the blades.
The selected ESC/motor pair must have compatible signal types. Default selection is P0.
`SpeedDrivenRotor`, `VoltageDrivenRotor`, their partial classes and
`VoltageDrivenMultirotor*` are removed. The former `PartialMultirotorPlant` equations
now reside directly in `MultirotorPlant` with public wrench/kinematics ports.

Both VIO compositions extend `Exports.FMU.CopterInterface` and reuse the same
`Adapters.VirtualIO.PwmInput`. `geometry.actuatorIndex` mapping belongs to the composer.
The manifest records `input_adapter.propulsion_input`, `command_meaning`, `mapping_owner`,
`propulsion_contract` and exact `propulsion_components` classes.
`--profile plant` accepts `normalizedCommand` and exposes P1 `dcVoltage` with an input
default; `--profile vio` owns a configured voltage parameter and keeps PWM/sensor IO.
`vio` is canonical; `fastdyn` remains a CLI/API alias with identical generated source.
Adapters contain only `PwmInput`/`SensorValues`; Exports.FMU owns IO and diagnostics.
The composer supplies plant pressure/temperature and connections. FMI2 and FMI3 use
these same compositions. Regenerate staged sources and FMUs after migration.

The `plant` and `vio` generated models both contain a `plant` component. The `plant` profile
forwards environmental inputs and exposes truth, measurements and wrench diagnostics;
computed hub wrenches stay inside the composition. Custom vehicle modifiers belong under `plant(...)`.

## P2 electromechanical propulsion

P2 selects `average_unidirectional` + `dynamic_dc_equivalent` + `quadratic_blade`.
Use `configs/quad_p2.toml` or `configs/hex_p2.toml`; their parameters are illustrative,
not calibrated motor/ESC data. P0/P1 remain empirical speed responses.

`Physical.Electrical.Passives.SeriesRL` composes MSL resistor and inductor with
bidirectional pins. `Physical.Mechanical.Rotational.RigidShaft` composes inertia and
viscous friction. `Systems.Propulsion.Motors.DynamicDCEquivalent` connects these
through MSL `RotationalEMF`. Its current and speed obey:

```text
L*der(i) = Vm - R*i - k*omega
(J + JLoad)*der(omega) = k*i - b*omega - loadTorque
loadTorque = kQ*omega^2
```

Explicit positive `motor.R` [ohm], `L` [H], `k` [N.m/A = V.s/rad] and `J` [kg.m2]
are required, together with explicit nonnegative `rotor.J` and `power.dcVoltage`.
`motor.b`, `i_start`, `omega_start` default to zero and must be nonnegative for P2.
Motor/rotor fields accept scalars or length-nRotors arrays. `rotor.J` is routed to
motor `JLoad`, not the aerodynamic blade. No physical motor defaults are guessed.
`k` uses one consistent DC-equivalent SI basis; a P1 fitted `Kv` or BLDC phase
constant must not be copied without converting/identifying the equivalent circuit.
There is no empirical time constant or speed cap in P2.

The nonregenerative ESC uses average modulation. At zero command, existing positive
inductor current circulates at zero terminal voltage until reaching zero. Only then
does the output block and float to back-EMF; no reverse current/active braking is
allowed. A positive command below back-EMF also remains blocked once current is zero.
The ESC's `coast` signal connects to motor `currentZero`: only at that natural
zero-current event does the motor project its actual inductor state to zero. It does
not reset current at command cutoff. `openCircuitVoltage=motors.backEMF` gives the
blocked terminal voltage while retaining a fixed current state. This avoids the
index-changing ideal-diode/inductor open circuit. These two auxiliary signals accompany
the conserving electrical pins; SeriesRL itself contains no unilateral constraint.

The ideal converter reports `dcCurrent=modulation*motorCurrent` in powered conduction,
and zero supply current in freewheel/coast. Public P2 diagnostics in both profiles are
`motorCurrent`, `dcCurrent`, `electromagneticTorque`, `loadTorque`, `electricalPower`,
`copperLoss`, `frictionLoss`, `loadPower`, and `storedEnergy` (per-rotor arrays).
`electricalPower = copperLoss + frictionLoss + loadPower + der(storedEnergy)` includes
both magnetic and rotational stored energy. Heat is accounted as a loss, without a
thermal state. A kinematic shaft angle may also be retained by the Modelica compiler.

`HubReaction` adds `-spinSign*(J+JLoad)*der(omega)` to the blade's aerodynamic moment.
The vehicle applies the mount rotation and CG moment arm once; do not also add a motor
housing reaction. This is a fixed-carrier axial approximation: carrier angular
acceleration feedback and gyroscopic coupling are excluded. Physical motor/propeller
mass belongs once in the vehicle mass budget; relative spin inertia is counted once
on the shaft. P2 does not add PWM ripple, phase commutation, PWM-cycle discontinuous
conduction, saturation, current limiting, battery/SOC, thermal dynamics or new aerodynamics.

Run `Examples.QuadElectromechanicalCoast` to inspect hover, cutoff and restart.
`python3 tools/probe_p2_export.py` builds and steps quad/hexa FMI2 models and compares
0.1 ms and 0.05 ms steps against native DASSL. Choose the simulation step with respect
to `L/R`; the example convergence result does not certify arbitrary parameter sets.

P2 native/FMI2 verification and the current Rumoca FMI3 compilation limitation
are recorded in [validation](validation.md#p2-electromechanical-validation-2026-10-12).

## Three-phase propulsion extension

P3/P4 reuse `Systems.Propulsion.Motors.ThreePhasePM`: three `SeriesRL` windings,
a lossless `PermanentMagnet3Phase` converter and `RigidShaft`. Their ESCs expose
conserving DC and ABC pins. A shared `Systems.Power.DCLink` supplies all rotors.
`Physical.Electrical.Drives.Control` owns six-step/coast policy and P4 PWM timing.
These are ESC-internal signal blocks: `Control` supplies commutation and gate
timing, `Bridges` supplies the power circuit, and `ESC` assembles both. The
top-level `Controllers` package is reserved for higher-level control laws.
P3/P4 here are known-position six-step variants; FOC remains future work.
The equations, ownership and configuration contract are documented in
[propulsion_modeling.md](propulsion_modeling.md).
