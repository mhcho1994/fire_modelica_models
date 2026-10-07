# Minimum continuous multicopter architecture

This implementation follows MULTICOPTER_INTERFACES v0.7 when the older port-contract
summary is ambiguous about frames. Active core contract: `ned_frd_continuous_v1`.
Contract IDs describe physical boundaries; profiles P0/R0 describe approximations.

## Composition and ownership

```text
PWM values -> PwmDemand -> demand[N] -> FirstOrderSpeed -> QuadraticBlade
                                                           |
static mass/mount records -> ChassisAssembly -> hub wrench rotation + CG r x F
                                                           |
                                                     RigidBody6DOF
                                                           |
                                      IMU / magnetometer / GNSS / barometer
```

`SpeedDrivenRotor` combines the motor and blade. Do not place another speed response
in front of it. The blade returns local hub force/moment; only the vehicle adds
`cross(rotorPosition_b, rotorForce_b)`. Gravity is applied only by the rigid body.
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

The only differential states are rotor speeds and rigid-body position, body velocity,
Hamilton quaternion and body angular velocity. Full inertia and quaternion dynamics
remain part of the minimum rigid-body contract. Sensors add no states. Fixed bias
parameters default to zero. Four-point spring/damper contact is enabled by default;
`landingGear(enabled=false)` removes it structurally. No body drag, sample/hold, sensor response filter,
noise, power accounting, gyro rotor reaction, inflow or coaxial interference is active.
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
| `rotor_wrench_v1` / SpeedDrivenRotor | demand (1; 1), airVelocity_r (3; m/s), omegaBody_r (3; rad/s) | force_r (3; N), moment_r (3; N.m), rotorSpeed (1; rad/s) | rotor local; hub, local +z thrust |
| R0 / QuadraticBlade | omega (1; rad/s), airVelocity_r (3; m/s), omegaBody_r (3; rad/s) | force_r (3; N), moment_r (3; N.m) | rotor hub; moment opposes spinSign |
| `rigid_body_wrench_v1` / RigidBody6DOF | force_b (3; N), moment_b (3; N.m), gravity_w (3; m/s2) | p_w (3; m), v_w/v_b (3; m/s), a_w/specificForce_b (3; m/s2), omega_b (3; rad/s), alpha_b (3; rad/s2), q_wb (4; 1), R_wb (3x3; 1) | NED world / FRD body at CG |
| LandingGearAssembly | p_w (3; m), v_b (3; m/s), omega_b (3; rad/s), R_wb (3x3), terrainPoint_w (3; m), terrainNormal_w (3), terrainVelocity_w (3; m/s) | force_b (3; N), moment_b (3; N.m), contact (nLegs; Boolean), gap (nLegs; m), normalForce (nLegs; N) | FRD wrench about combined CG; NED plane/kinematics |
| `imu_measurement_v1` / IMU.Sensor | R_wb (3x3; 1), a_w/gravity_w (3; m/s2), omega_b (3; rad/s), alpha_b (3; rad/s2) | acceleration (3; m/s2), gyro (3; rad/s) | sensor local at rigid mount |
| `mag_measurement_v1` / Magnetometer.Sensor | R_wb (3x3; 1), magneticField_w (3; T) | magneticField (3; T) | sensor local |
| `gnss_measurement_v1` / GNSS.Sensor | position_w (3; m), velocity_w (3; m/s) | position (3; m), velocity (3; m/s) | NED, antenna |
| `barometer_measurement_v1` / Barometer.Sensor | ambientPressure (1; Pa), ambientTemperature (1; K), altitude_w (1; m), climbRate_w (1; m/s) | pressure, temperature, altitude, climbRate (matching units) | scalar; altitude/climb rate Up-positive |

Motor parameters are `tau>0`, `omegaMax>0`, `0<=omega_start<=omegaMax`.
Rotor `spinSign` is +1/-1 about local +z. `moment_r` is the signed aerodynamic hub
moment, not a shaft `loadTorque`. No electrical or shaft-load port is advertised.
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

`SensorValues` and `MultirotorValueAdapter` apply only sensor mount rotation to IMU/mag;
GNSS passes through as NED. `CopterInterface` additionally converts Tesla to Gauss,
Kelvin to Celsius and NED to approximate geodetic coordinates, and computes heading
as atan2(R_wb[2,1],R_wb[1,1]). It uses constant pressure/temperature in this profile.
The value/FMU adapter PWM range is 1000–2000 us; the FastDyn composer boundary keeps
1100–1900 us defaults, configurable through `[interface]`.

Composer schema 3 and manifest version 2 explicitly record frames, core contract,
P0/R0, continuous acquisition, no drag, and the selected contact profile.
`[landing_gear]` resolves to `LandingGear.Parameters`; the manifest sets
`physics.contact="four_point_spring_damper"` and `event_requirements.ground_contact=true`
when enabled, otherwise `"none"` and `false`. Archived files are excluded from
both staging and source hashing. Existing generated artifacts must be regenerated.
