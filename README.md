# FIRE Minimal Multicopter Models

[Korean version](README_ko.md)

The active package implements the initial execution scope defined in
`MULTICOPTER_INTERFACES.md` v0.8 and `261006_quadrotor-fidelity-port-contract.md`.
The baseline consists of **continuous P0 motors + R0 rotors + a single 6DOF rigid body
+ ideal sensors + four-point compliant landing gear**, using SI units, a NED world frame, and FRD chassis/body frames.

- `Vehicles.Copter.MultirotorPlant`: sums the hub forces and moments of N rotors about the CG.
- `Vehicles.Copter.MultirotorWithSensors`: adds an IMU, magnetometer, GNSS, and barometer.
- `Vehicles.Copter.Presets`: QuadX, HexaX, OctoX, and CoaxialX8. The coaxial preset changes only the layout; aerodynamic interference is not modeled.
- `Physical.Mechanical.Chassis`: computes mass, CG, and the full inertia tensor from aggregate properties or a static assembly of parts. `LandingGear` reuses the archived point-contact models with FRD mounts and a NED ground normal.
- `Adapters.VirtualIO`: provides continuous PWM value normalization, sensor-local to FRD conversion, NED outputs, and an FMU boundary.

`demand` is a **target rotor-speed ratio, not a thrust ratio**. Motor lag is applied once.
Sensors use continuous ideal measurement equations with zero bias by default. There is no
sampling, hold, timestamp, or response filter. Landing gear is enabled by default and supports
the vehicle on a stationary horizontal plane. Set `landingGear(enabled=false)` for freefall
through the plane. P1 adds an ideal voltage-only ESC and voltage-to-speed surrogate.
General terrain, drag, motor current/torque, battery, and protocol models remain outside the active path.

## P0 and P1 propulsion

P0 remains the default: existing `quad.toml`, `hex.toml`, `SpeedDrivenRotor` and
the plant `demand` port retain their target-speed contract. Select `models.rotor="voltage_driven"`
with `configs/quad_p1.toml` or `configs/hex_p1.toml` for P1:

```text
PWM number -> VirtualIO.PwmInput -> normalizedCommand -> escCommand -> IdealVoltage ESC
                                                       + dcVoltage
                                                            |
                                                  VoltageToSpeed -> QuadraticBlade
```

`Physical.Electrical.Drives.ESC.IdealVoltage` computes
`equivalentVoltage = clamp(escCommand,0,1)*dcVoltage` without states.
`Systems.Propulsion.ResponseModels.VoltageToSpeed` owns the single speed state:
`der(omega) = (Kv*equivalentVoltage-omega)/tauOmega`. The command selects unipolar
voltage modulation. It is not a target-speed command despite the same numeric range.
P1 has no current, power accounting, load feedback, reverse rotation or active braking.
Zero voltage gives exponential coast-down. There is no additional speed cap.

P1 requires explicit `motor.Kv` in **(rad/s)/V** and nonnegative `power.dcVoltage` [V].
Convert rpm/V by multiplying by `2*pi/60`. `tauOmega` defaults to 0.03 s; motor fields
accept a scalar or one value per rotor. P0's `omegaMax`/`tau` and P1's `Kv`/`tauOmega`
are deliberately separate and cross-profile fields are rejected. With `Kv*Vnom=omegaMax`,
equal time constants and initial speeds, nominal P1 matches P0 under equal numeric commands.
The examples use 16 V and 62.5 (rad/s)/V as illustrative coefficients, not aircraft calibration.

`Examples.QuadVoltageSag` starts at hover and lowers the supply from 16 V to 12 V at 1 s.
The FastDyn config `configs/copter462_fire_p1.toml` (in the parent FastDyn repository)
uses an internal fixed supply and preserves the external PWM/sensor FMI boundary.
`configs/copter462_fire_p0.toml` selects P0 (renamed from `copter462_fire.toml`). For standalone `plant` composition,
`escCommand` and `dcVoltage` are input ports; the configured voltage is an input default.
When running that model directly as a simulation/FMU, set those inputs explicitly or
instantiate it in a harness. No current or battery-SOC port is exported.

## Common VirtualIO boundary

Both P0 and P1 extend `Adapters.VirtualIO.CopterInterface` and use the same `PwmInput`.
The adapter continuously clamps numeric pulse width to `normalizedCommand` in [0,1].
The composer binds it to P0 `plant.demand` (target speed ratio) or P1 `plant.escCommand`
(voltage modulation). Channel-to-rotor mapping stays in the plant.
`[interface] pwm_min/pwm_max` defaults to 1100/1900 us; standalone value/FMU adapters
keep 1000/2000 us. External PWM/sensor fields and units remain unchanged.

Use `--profile vio`; `--profile fastdyn` remains an alias and emits canonical `vio`
metadata. Modelica packages `Adapters.VIO` and `Adapters.FastDyn` are removed;
update source references to `Adapters.VirtualIO` and replace `commands.demand` or
`commands.escCommand` with `commands.normalizedCommand` at the appropriate plant binding.
Standalone `PwmInput` defaults to 1000/2000 us; a migrated legacy P1 PWM adapter must
explicitly pass 1100/1900 us (the shared CopterInterface already does so).
Regenerate composed sources and FMUs to remove old staged packages and update the cache.

## Running the models

```sh
omc tools/simulate_landing_gear.mos
python3 tools/verify.py
FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p test_composer.py
python3 tools/probe_export.py
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- \
  check configs/quad.toml --profile vio
```

OpenModelica and Modelica 4.0.0 are required. The FMU probe checks FMI 2.0 Co-Simulation on Linux.
`Examples.QuadHover`, `HexaHover`, `OctoHover`, and `CoaxialHover` are open-loop examples
whose initial rotor speeds balance gravity. They do not include a flight controller.

## Four-point landing gear

Load `package.mo` in OMEdit and simulate `Examples.QuadGroundCycle` for 6 seconds,
or run the `omc` command above from the library root. The example drops from 2 cm foot
clearance, settles, applies symmetric thrust at 2 s, cuts the motors at 2.7 s, and lands again.
Plot `altitude`, `normalLoad`, `contactCount`, and `acceleration_frd[3]`.
The command writes `build/landing-gear/QuadGroundCycle_res.csv`.

`vehicle.landingGear` is a `Physical.Mechanical.Chassis.LandingGear.Parameters` record:

| Parameter | Default / meaning |
|---|---|
| `enabled` | `true`; `false` removes all contacts at translation |
| `position_C[4,3]` | X/Y = ±0.17 m, Z = +0.10 m; FRD tips relative to chassis C |
| `groundZ` | 0 m; horizontal plane's NED Down coordinate |
| `stiffness`, `damping` | 3000 N/m and 150 N·s/m per foot |
| `tangentialDamping`, `frictionCoefficient` | 25 N·s/m, 0.6; viscous friction capped by normal load |

The existing rigid body supplies the sprung mass; gear adds no mass or motion states.
Account for physical leg mass in the aggregate budget or `additionalParts` exactly once.
Every tip is rebased by `chassis.cg_C`. The outward ground normal is `{0,0,-1}`.
Normal force is `max(0, k*max(0,-gap) - c*normalVelocity)` during contact;
friction is dissipative and limited to `mu*normalForce`, with no static-stick constraint.
Contact forces and their `r × F` moments enter the body once, alongside rotor and external loads.
Touchdown/liftoff remain Modelica events; there are no velocity resets or arming conditions.

For a level 1.5 kg quad with four 3000 N/m springs, static compression is
`mg/(4k) = 1.226 mm`, CG altitude is 0.098774 m, and FRD accelerometer Z is −9.80665 m/s².
Standalone plants retain `p_start={0,0,-1}`. Bundled quad/hexa configurations explicitly use
`p_start={0,0,-0.12}` for 2 cm initial clearance with their default CG and mounts.
For a shifted CG or custom attitude, choose the initial CG position using `p_tip = p_CG + R_wb*(position_C-cg_C)`.
Rotor count does not change the four-foot assembly.

Composer schema 3 accepts the optional `[landing_gear]` table with the record fields above:

```toml
[landing_gear]
enabled = true
groundZ = 0.0
stiffness = 3000.0
damping = 150.0
tangentialDamping = 25.0
frictionCoefficient = 0.6
```

## Coordinate frames and configuration migration

`R_ab` represents the attitude of frame b relative to frame a, with component conversion
`v_a = R_ab*v_b`. `q_wb={w,x,y,z}` is the Hamilton quaternion for the same attitude.
The default vehicle preserves the previous East heading with
`q_start={sqrt(0.5),0,0,sqrt(0.5)}`. A position of `p_start={0,0,-1}` is 1 m above the
ground origin. The identity quaternion represents a North heading.
The rotor-local +z thrust axis and sensor-local axes are preserved, so the default
mounting matrices are `diag(1,-1,-1)`. Set `R_bImu=identity(3)` for an IMU aligned with body FRD.

TOML configurations must use **schema_version=3**. Legacy schemas 1/2, the old `[ground]` settings and `geometry.nLegs`,
sample periods, and `first_order` sensor responses are rejected. Embedded physics tables
in existing FastDyn host TOML files require the same migration. Convert positions, vectors,
inertia, mounts, and initial attitudes together; changing only the schema number is insufficient.
See [architecture](docs/architecture.md) for the transformations and ports.

Previous fixed-wing, rover, terrain, sampling, filter, and communication paths, including
the original ENU/FLU landing-gear implementation, are preserved in [deprecated](deprecated/README.md). Do not load the old
classes and the new NED/FRD classes together under the same namespace. Historical validation
records do not certify the new models.

See the [validation scope](docs/validation.md), [sensor contracts](Systems/Sensing/README.md),
and [composition tool](tools/fire-compose/README.md) for details.
The P1 extension additionally exercises Rumoca FMI 3 packaging and real Co-Simulation
steps through the parent FastDyn integration test. RBC linking and actual firmware/flight
validation remain outside that test's scope.
