within FIRE_Modelica.Tests;

model SensorLegacySampling "Compare all four legacy pre-buffer sensors with settled sampled outputs"
  parameter Real pressureSlope(unit="Pa/s") = 1;
  parameter Real temperatureSlope(unit="K/s") = 1;
  FIRE_Modelica.Systems.Sensing.SensorSuite suite(
    imuSamplePeriod=0.01, magnetometerSamplePeriod=0.02,
    gnssSamplePeriod=0.1, barometerSamplePeriod=0.05,
    accelBias={0.1,0.2,0.3}, gyroBias={0.01,0.02,0.03},
    magBias={1e-6,2e-6,3e-6}, positionBias={1,2,3},
    velocityBias={0.1,0.2,0.3}, pressureBias=10, altitudeBias=2);
  FIRE_Modelica.Systems.Sensing.IMU.LowFidelity imu(
    samplePeriod=0.01, accelBias={0.1,0.2,0.3}, gyroBias={0.01,0.02,0.03});
  FIRE_Modelica.Systems.Sensing.Magnetometer.LowFidelity magnetometer(
    samplePeriod=0.02, magBias={1e-6,2e-6,3e-6});
  FIRE_Modelica.Systems.Sensing.GNSS.LowFidelity gnss(
    samplePeriod=0.1, positionBias={1,2,3}, velocityBias={0.1,0.2,0.3});
  FIRE_Modelica.Systems.Sensing.Barometer.LowFidelity barometer(
    samplePeriod=0.05, pressureBias=10, altitudeBias=2);
  Modelica.Blocks.Sources.RealExpression magneticSource[3](y=suite.magneticField_w);
  Modelica.Blocks.Sources.RealExpression pressureSource(y=suite.pressure);
  Modelica.Blocks.Sources.RealExpression temperatureSource(y=suite.temperature);
  Real magneticError[3];
  Real accelerationError[3];
  Real gyroError[3];
  Real positionError[3];
  Real velocityError[3];
equation
  suite.p_w = {time,2*time,3*time};
  suite.v_w = {1,2,3};
  suite.R_wb = identity(3);
  suite.a_w = {time,2*time,0};
  suite.omega_b = {time,2*time,3*time};
  suite.alpha_b = {1,2,3};
  suite.gravity_w = {0,0,-9.81};
  suite.magneticField_w = {1e-5+time*1e-6,2e-5,3e-5};
  suite.pressure = 101325 + pressureSlope*time;
  suite.temperature = 288.15 + temperatureSlope*time;
  imu.velocityBody = suite.v_w;
  imu.accelerationBody = suite.a_w - suite.gravity_w;
  imu.euler = zeros(3);
  imu.rates = suite.omega_b;
  magnetometer.euler = zeros(3);
  connect(magneticSource.y, magnetometer.world.magneticField);
  gnss.position = suite.p_w;
  gnss.velocityWorld = suite.v_w;
  barometer.position = suite.p_w;
  barometer.velocityWorld = suite.v_w;
  connect(pressureSource.y, barometer.world.pressure);
  connect(temperatureSource.y, barometer.world.temperature);
  accelerationError = {imu.ax,imu.ay,imu.az} - suite.measurements.acceleration;
  gyroError = {imu.p,imu.q,imu.r} - suite.measurements.gyro;
  magneticError = {magnetometer.mx,magnetometer.my,magnetometer.mz} - suite.measurements.magneticField;
  positionError = {gnss.x,gnss.y,gnss.z} - suite.measurements.position;
  velocityError = {gnss.vx,gnss.vy,gnss.vz} - suite.measurements.velocity;
  when sample(0.001, 0.005) then
    assert(sum(abs(accelerationError)) + sum(abs(gyroError)) < 1e-10,
      "Legacy IMU settled sample/hold behavior changed");
    assert(sum(abs(magneticError)) < 1e-12,
      "Legacy magnetometer settled sample/hold behavior changed");
    assert(sum(abs(positionError)) + sum(abs(velocityError)) < 1e-10,
      "Legacy GNSS settled sample/hold behavior changed");
    assert(abs(barometer.pressure-suite.measurements.pressure) < 1e-8 and
      abs(barometer.temperature-suite.measurements.temperature) < 1e-10 and
      abs(barometer.baroAltitude-suite.measurements.altitude) < 1e-10 and
      abs(barometer.baroClimbRate-suite.measurements.climbRate) < 1e-10,
      "Legacy barometer settled sample/hold behavior changed");
  end when;
  annotation(experiment(StopTime=0.225, Interval=0.001, Tolerance=1e-8));
end SensorLegacySampling;
