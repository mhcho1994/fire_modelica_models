# Sensing devices and optional assemblies

The canonical sensor families are `Systems.Sensing.IMU`, `Magnetometer`, `GNSS`,
and `Barometer`. The former top-level `Sensors` package has been removed; update
external class references to `fire_modelica_models.Systems.Sensing` as well.

`SensorSuite` is an optional default assembly of four sensors. A vehicle or
experiment may instantiate individual sensors directly, including multiple
instances of a family. `Examples.QuadImuOnly` demonstrates direct plant/IMU
composition, CG-relative mounting, and a sensor-owned sampling clock. Individual
sensor models do not depend on the suite, and the suite does not duplicate their
measurement or sampling equations.

## One public assembly per sensor family

Use `IMU.Sensor`, `Magnetometer.Sensor`, `GNSS.Sensor`, or `Barometer.Sensor`.
The earlier `IMU.MountedSampled` and other families' `Sampled` classes have been
replaced, with no aliases. Update external references to `Sensor`. The default
configuration preserves the previous equations, units, frames, biases and clocks.

Each `Sensor` composes this chain:

```text
Measurement -> Response -> bias -> sampling/hold -> named outputs + sampleTime
```

- `Measurement` is a replaceable model constrained by the family's
  `BaseClasses.PartialMeasurement`. Its default `IdealMeasurement` computes the
  continuous observables. IMU includes specific force and rigid lever-arm terms;
  magnetometer includes the mount rotation. GNSS inputs are already antenna motion.
- `Response` is a replaceable model constrained by
  `ResponseModels.PartialResponse`. `Ideal` passes channels through;
  `FirstOrder` applies independent continuous first-order dynamics. Its `tau`
  array is in seconds: positive entries add lag, zero entries bypass that channel,
  and negative entries are rejected. Filter states initialize at their input.
- `Sensor` owns output biases and the acquisition clock. A replacement component
  must preserve channel order, units and frames, and must not add its own output
  bias or sampling if those functions are still performed by `Sensor`.

`BaseClasses` holds family-specific observable contracts and implementations;
these helpers are not separate public fidelity levels. Reusable MEMS, AFE and
power components, when implemented, belong to `Physical`; digital components
belong to `Logical`. The current empirical response belongs to `Systems.Sensing`,
and does not represent a physical circuit or a pure transport delay.

| Family | Response channel order |
|---|---|
| IMU | acceleration xyz [m/s2], gyro xyz [rad/s], both in sensor axes |
| Magnetometer | magnetic field xyz [T], in sensor axes |
| GNSS | antenna position xyz [m], velocity xyz [m/s], world ENU |
| Barometer | pressure [Pa], temperature [K], geometric altitude [m], climb rate [m/s] |

Channels have different SI units; the shared response vector performs no unit or
frame conversion. Its dimension is fixed by each assembly (`final n`).

## Configuration examples

A directly placed IMU with different acceleration/gyro time constants:

```modelica
fire_modelica_models.Systems.Sensing.IMU.Sensor imu(
  samplePeriod=0.0025,
  accelBias={0.01,0,0},
  redeclare model Response =
    fire_modelica_models.Systems.Sensing.ResponseModels.FirstOrder(
      tau={0.01,0.01,0.01,0.005,0.005,0.005}));
```

Connect its motion/gravity inputs as shown in `Examples.QuadImuOnly`.
`Examples.QuadImuResponse` is a runnable version using this response configuration.
The same modifier works inside the optional suite:

```modelica
fire_modelica_models.Systems.Sensing.SensorSuite sensors(
  imu(redeclare model Response =
    fire_modelica_models.Systems.Sensing.ResponseModels.FirstOrder(
      tau={0.01,0.01,0.01,0,0,0})));
```

For a vehicle, put that modifier on `MultirotorWithSensors.sensors.imu`.
To replace measurement physics, use `redeclare model Measurement = MyMeasurement`,
where `MyMeasurement` extends that family's `BaseClasses.PartialMeasurement`.
`Tests.SensorReconfiguration` demonstrates a custom IMU measurement replacement,
all four response configurations, zero-time-constant channels and direct composition.

`redeclare`, channel structure and the zero/nonzero `tau` choices are compile-time
configuration. Biases, sample periods and time constants are parameters selected
before execution; exported FMU variability determines whether a particular value
can be set during initialization. This implementation does not support changing
ODR/range through registers at runtime.

Modeling scope, element fidelity and external interface remain independent.
Validated cumulative presets can represent increasing fidelity when their
included behavior is explicit, but presets are not required for every delay or
parameter choice. A future register/SPI interface may need a different outer
assembly reusing the same internals. Noise, drift, pure transport delay, detailed
MEMS/AFE/ADC, registers/FIFO and EMI are not implemented by this refactor.

## Measurement and timing contract

The four existing `LowFidelity` classes retain their equations, mounting, biases,
sampling periods, and legacy expandable buses under the new family paths.
Fixed-wing and rover assemblies still use them. Consolidating these classes
with the new assemblies requires a separate migration of those vehicles' physical
input meanings, frames, and bus contracts; this package move does not remove them.

`Systems.Sensing.SensorSuite` assembles the four configurable `Sensor` models with a
`SensorMeasurements` output record. Position and velocity are measured at the
GNSS antenna; acceleration and angular velocity are resolved in the IMU frame;
magnetic field is resolved in the magnetometer frame. Attitude and other body
truth belong to `VehicleTruth`, not to a sensor output.

All mount matrices map sensor coordinates **to body coordinates**. Sensor
offsets are relative to the complete vehicle's center of mass. The IMU includes
both angular-acceleration and centripetal lever-arm terms. These variants assume
mounts rigidly fixed to the body; a moving mount requires the actual sensor's
local absolute motion instead of this fixed-offset formula.

Each representative `Sensor` acquires at `sample(0, samplePeriod)`, adds its bias at acquisition,
and holds its output until the next acquisition. Timestamps describe acquisition
time, not transmission or reception. Sample periods must be positive. New
variants explicitly initialize discrete states; the settled first sample is
available at the right limit of the time-zero event. Event output files may
contain multiple rows with the same timestamp: use the final row at that time
when comparing settled measurements.

The legacy `output = pre(buffer)` equations occur outside their sampling `when`.
An actual legacy GNSS probe using OpenModelica 1.26.7, a period of 0.2 s,
`position[1] = 10 + time`, and a bias of 1 produced settled outputs 11 at 0 s,
11.2 at 0.2 s, and 11.4 at 0.4 s, held between samples. It did **not** add an
extra full sample delay. Initial pre-event rows contained the buffer start value.
The unchanged legacy buffers can produce OpenModelica's existing warning about
underspecified initialization; new variants set their initial discrete values
explicitly. This observation is specific to the verified execution path, and
does not establish event behavior in an exported FMU or a different runtime.

Barometer `Sensor` with its default components intentionally retains the low-fidelity separation between
ambient pressure/temperature and geometric altitude/climb rate. Constant
environment pressure therefore stays constant even when geometric altitude
changes. It does not implement an atmospheric pressure-height law, pressure-to-
altitude inversion, temperature compensation, or GNSS geodetic conversion.

The following assertion models exercise the contract:

- `Tests.SensorPhysics`: freefall, supported specific force, rotation, lever arm,
  and bias.
- `Tests.SensorSampling`: time-zero right-limit samples, independent rates,
  holding, acquisition timestamps, mounting, and ambient quantities.
- `Tests.SensorReconfiguration`: custom measurement replacement, all four first-order
  responses against analytical ramp solutions, initialization, bypass, and hold.
- `Tests.SensorLegacySampling`: all four unchanged legacy families compared with
  the new settled sampled measurements between event instants.

These models were checked and simulated in OpenModelica 1.26.7. The default
`Ideal` configuration is also exercised by the integrated FMU probe described in
[export validation](../../docs/export_validation.md). Reconfigured responses were
validated in native simulation; their FMU execution and Rumoca execution remain
separate compatibility checks.
