# Continuous ideal sensors

Each `Sensor` evaluates named, unit-typed outputs at the current simulation time.
There is no sample period, sampleTime, hold, device response, noise or transport state.
Fixed biases default to zero and are expressed in the output frame.
The earlier Measurement/Response arrays, first-order filters, sampled wrappers and
legacy LowFidelity models are preserved in `deprecated/legacy_enu_flu/fire_modelica_models`.

- IMU: `transpose(R_bs) * (transpose(R_wb)*(a_w-gravity_w) + alpha_b x r_b + omega_b x (omega_b x r_b))`; gyro is `transpose(R_bs)*omega_b`.
- Magnetometer: `transpose(R_bs)*transpose(R_wb)*magneticField_w` (Tesla).
- GNSS: antenna position and velocity already resolved in NED.
- Barometer: ambient Pa/K and geometric Up-positive altitude/climb rate, without pressure-height inversion.

`R_ab` represents frame b's attitude relative to a; component conversion is `v_a=R_ab*v_b`.
Standalone sensor `R_bs=identity(3)` aligns its local axes with body FRD.
Vehicle geometry defaults to `R_bImu=R_bMag=diag(1,-1,-1)` to retain the earlier local axes.
Thus default local IMU hover z is +g, while the FastDyn body-FRD output z is -g.
SensorSuite is optional; `Examples.QuadImuOnly` connects a standalone IMU to the plant.
The suite owns rigid GNSS/barometer lever-arm position/velocity conversion and converts
NED z to Up-positive scalar height/rate. There are no dummy orientation ports on scalar
barometer or NED GNSS measurements.
