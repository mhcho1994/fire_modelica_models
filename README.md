# FIRE Minimal Multicopter Models

[Korean version](README_ko.md)

The active package implements the initial execution scope defined in
`MULTICOPTER_INTERFACES.md` v0.7 and `261006_quadrotor-fidelity-port-contract.md`.
The baseline consists of **continuous P0 motors + R0 rotors + a single 6DOF rigid body
+ ideal sensors + four-point compliant landing gear**, using SI units, a NED world frame, and FRD chassis/body frames.

- `Vehicles.Copter.MultirotorPlant`: sums the hub forces and moments of N rotors about the CG.
- `Vehicles.Copter.MultirotorWithSensors`: adds an IMU, magnetometer, GNSS, and barometer.
- `Vehicles.Copter.Presets`: QuadX, HexaX, OctoX, and CoaxialX8. The coaxial preset changes only the layout; aerodynamic interference is not modeled.
- `Physical.Mechanical.Chassis`: computes mass, CG, and the full inertia tensor from aggregate properties or a static assembly of parts. `LandingGear` reuses the archived point-contact models with FRD mounts and a NED ground normal.
- `Adapters.FastDyn`: provides continuous PWM value normalization, sensor-local to FRD conversion, NED outputs, and an FMU boundary.

`demand` is a **target rotor-speed ratio, not a thrust ratio**. Motor lag is applied once.
Sensors use continuous ideal measurement equations with zero bias by default. There is no
sampling, hold, timestamp, or response filter. Landing gear is enabled by default and supports
the vehicle on a stationary horizontal plane. Set `landingGear(enabled=false)` for freefall
through the plane. General terrain, drag, electrical motor/ESC, battery, and protocol models
remain outside the active path.

## Running the models

```sh
omc tools/simulate_landing_gear.mos
python3 tools/verify.py
FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p test_composer.py
python3 tools/probe_export.py
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- \
  check configs/quad.toml --profile fastdyn
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
Rumoca RBC linking, FMI 3 packaging, and actual FastDyn firmware integration are outside
this change's validation scope.
