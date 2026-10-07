within fire_modelica_models.Vehicles.Copter;
model MultirotorWithSensors "Value-level sampled sensor profile; no protocol/firmware driver"
  extends MultirotorPlant;
  parameter Boolean sampledSensors = true;
  parameter Real imuSamplePeriod = 0.0025;
  parameter Real magnetometerSamplePeriod = 0.02;
  parameter Real gnssSamplePeriod = 0.2;
  parameter Real barometerSamplePeriod = 0.02;
  input Real magneticField_w[3] = {20e-6,0,-45e-6};
  input Real pressure = 101325 "Constant ambient pressure baseline [Pa]";
  input Real temperature = 288.15 "K";
  Systems.Sensing.SensorSuite sensors(
    sampled=sampledSensors,
    rImu_b=geometry.imuPosition_C-chassis.cg_C,R_bImu=geometry.R_bImu,R_bMag=geometry.R_bMag,
    rGnss_b=geometry.gnssPosition_C-chassis.cg_C,rBarometer_b=geometry.barometerPosition_C-chassis.cg_C,
    imuSamplePeriod=imuSamplePeriod,magnetometerSamplePeriod=magnetometerSamplePeriod,
    gnssSamplePeriod=gnssSamplePeriod,barometerSamplePeriod=barometerSamplePeriod);
  output Interfaces.SensorMeasurements measurements;
initial equation
  assert(max(abs(transpose(geometry.R_bImu)*geometry.R_bImu-identity(3)))<1e-9
    and abs(Modelica.Math.Matrices.det(geometry.R_bImu)-1)<1e-9,"IMU mount must be a proper rotation");
  assert(max(abs(transpose(geometry.R_bMag)*geometry.R_bMag-identity(3)))<1e-9
    and abs(Modelica.Math.Matrices.det(geometry.R_bMag)-1)<1e-9,"Magnetometer mount must be a proper rotation");
equation
  sensors.p_w=body.p_w;
  sensors.v_w=body.v_w;
  sensors.R_wb=body.R_wb;
  sensors.a_w=body.a_w;
  sensors.omega_b=body.omega_b;
  sensors.alpha_b=body.alpha_b;
  sensors.gravity_w=gravity_w;
  sensors.magneticField_w=magneticField_w;
  sensors.pressure=pressure;
  sensors.temperature=temperature;
  measurements=sensors.measurements;
end MultirotorWithSensors;
