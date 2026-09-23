within fire_modelica_models.Systems.Sensing;

model SensorSuite "Optional four-sensor assembly; each sensor owns its sampling and measurement behavior"
  parameter Boolean sampled = true "Sample and hold, or continuous response outputs";
  parameter Real rImu_b[3] = zeros(3) "IMU displacement from total CG [m]";
  parameter Real R_bImu[3,3] = identity(3) "IMU frame to body";
  parameter Real R_bMag[3,3] = identity(3) "Magnetometer frame to body";
  parameter Real rGnss_b[3] = zeros(3) "Antenna displacement from total CG [m]";
  parameter Real rBarometer_b[3] = zeros(3) "Barometer displacement from total CG [m]";
  parameter Real imuSamplePeriod = 0.0025;
  parameter Real magnetometerSamplePeriod = 0.02;
  parameter Real gnssSamplePeriod = 0.2;
  parameter Real barometerSamplePeriod = 0.02;
  parameter Real accelBias[3] = zeros(3);
  parameter Real gyroBias[3] = zeros(3);
  parameter Real magBias[3] = zeros(3);
  parameter Real positionBias[3] = zeros(3);
  parameter Real velocityBias[3] = zeros(3);
  parameter Real pressureBias = 0;
  parameter Real temperatureBias = 0;
  parameter Real altitudeBias = 0;
  input Real p_w[3];
  input Real v_w[3];
  input Real R_wb[3,3];
  input Real a_w[3];
  input Real omega_b[3];
  input Real alpha_b[3];
  input Real gravity_w[3];
  input Real magneticField_w[3];
  input Real pressure "Ambient pressure supplied by the environment [Pa]";
  input Real temperature "Ambient temperature supplied by the environment [K]";
  output fire_modelica_models.Interfaces.SensorMeasurements measurements;

  IMU.Sensor imu(
    sampled=sampled, samplePeriod=imuSamplePeriod, r_b=rImu_b, R_bs=R_bImu,
    accelBias=accelBias, gyroBias=gyroBias);
  Magnetometer.Sensor magnetometer(
    sampled=sampled, samplePeriod=magnetometerSamplePeriod, R_bs=R_bMag, bias=magBias);
  GNSS.Sensor gnss(
    sampled=sampled, samplePeriod=gnssSamplePeriod, positionBias=positionBias, velocityBias=velocityBias);
  Barometer.Sensor barometer(
    sampled=sampled, samplePeriod=barometerSamplePeriod, pressureBias=pressureBias,
    temperatureBias=temperatureBias, altitudeBias=altitudeBias);
protected
  Real barometerPosition_w[3];
  Real barometerVelocity_w[3];
equation
  imu.R_wb = R_wb;
  imu.a_w = a_w;
  imu.gravity_w = gravity_w;
  imu.omega_b = omega_b;
  imu.alpha_b = alpha_b;
  magnetometer.R_wb = R_wb;
  magnetometer.magneticField_w = magneticField_w;
  gnss.position_w = p_w + R_wb * rGnss_b;
  gnss.velocity_w = v_w + R_wb * cross(omega_b, rGnss_b);
  barometerPosition_w = p_w + R_wb * rBarometer_b;
  barometerVelocity_w = v_w + R_wb * cross(omega_b, rBarometer_b);
  barometer.ambientPressure = pressure;
  barometer.ambientTemperature = temperature;
  barometer.altitude_w = barometerPosition_w[3];
  barometer.climbRate_w = barometerVelocity_w[3];
  measurements.acceleration = imu.acceleration;
  measurements.gyro = imu.gyro;
  measurements.magneticField = magnetometer.magneticField;
  measurements.position = gnss.position;
  measurements.velocity = gnss.velocity;
  measurements.pressure = barometer.pressure;
  measurements.temperature = barometer.temperature;
  measurements.altitude = barometer.altitude;
  measurements.climbRate = barometer.climbRate;
  measurements.imuSampleTime = imu.sampleTime;
  measurements.magnetometerSampleTime = magnetometer.sampleTime;
  measurements.gnssSampleTime = gnss.sampleTime;
  measurements.barometerSampleTime = barometer.sampleTime;
end SensorSuite;
