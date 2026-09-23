within FIRE_Modelica.Tests;

model SensorSampling "Right-limit initial samples, independent clocks, holding, bias, and antenna mount"
  parameter Real temperatureSlope(unit="K/s") = 1;
  FIRE_Modelica.Systems.Sensing.SensorSuite suite(
    imuSamplePeriod=0.01, magnetometerSamplePeriod=0.02,
    gnssSamplePeriod=0.1, barometerSamplePeriod=0.05,
    R_bMag=[0,-1,0; 1,0,0; 0,0,1], rGnss_b={1,0,0},
    rBarometer_b={0,0,0.5}, accelBias={0.1,0,0}, magBias={1e-6,0,0},
    positionBias={1,0,0}, pressureBias=10, temperatureBias=1, altitudeBias=2);
equation
  suite.p_w = {10+time,0,time};
  suite.v_w = {1,0,1};
  suite.R_wb = identity(3);
  suite.a_w = {time,0,0};
  suite.omega_b = {0,0,2};
  suite.alpha_b = zeros(3);
  suite.gravity_w = {0,0,-9.81};
  suite.magneticField_w = {1e-5+time*1e-6,2e-5,3e-5};
  suite.pressure = 101325 + 100*time;
  suite.temperature = 288.15 + temperatureSlope*time;
  when time >= 0.001 then
    assert(abs(suite.measurements.imuSampleTime) < 1e-10 and
      abs(suite.measurements.gnssSampleTime) < 1e-10,
      "First acquisition must occur at time zero");
    assert(abs(suite.measurements.acceleration[1] - 0.1) < 1e-10 and
      abs(suite.measurements.position[1] - 12) < 1e-10,
      "First right-limit sample must include bias and antenna offset");
    assert(sum(abs(suite.measurements.velocity - {1,2,1})) < 1e-10,
      "Antenna velocity must include omega cross lever arm");
    assert(abs(suite.measurements.altitude - 2.5) < 1e-10,
      "Barometer geometric altitude must include mount height and bias");
  end when;
  when time >= 0.035 then
    assert(abs(suite.measurements.imuSampleTime - 0.03) < 1e-10 and
      abs(suite.measurements.magnetometerSampleTime - 0.02) < 1e-10 and
      abs(suite.measurements.gnssSampleTime) < 1e-10 and
      abs(suite.measurements.barometerSampleTime) < 1e-10,
      "Independent sensor clocks or hold semantics are incorrect");
    assert(abs(suite.measurements.acceleration[1] - 0.13) < 1e-10 and
      abs(suite.measurements.position[1] - 12) < 1e-10 and
      abs(suite.measurements.pressure - 101335) < 1e-8,
      "Measurements must hold the last acquisition, without an extra sample delay");
    assert(sum(abs(suite.measurements.magneticField - {2.1e-5,-1.002e-5,3e-5})) < 1e-12,
      "Magnetometer rotation, Tesla bias, or sampling is incorrect");
  end when;
  when time >= 0.115 then
    assert(abs(suite.measurements.imuSampleTime - 0.11) < 1e-10 and
      abs(suite.measurements.magnetometerSampleTime - 0.1) < 1e-10 and
      abs(suite.measurements.gnssSampleTime - 0.1) < 1e-10 and
      abs(suite.measurements.barometerSampleTime - 0.1) < 1e-10,
      "Coincident sensor acquisitions did not settle independently");
    assert(abs(suite.measurements.position[1] - 12.1) < 1e-10 and
      abs(suite.measurements.pressure - 101345) < 1e-8 and
      abs(suite.measurements.temperature - 289.25) < 1e-10,
      "Latest samples or environmental biases are incorrect");
  end when;
  annotation(experiment(StopTime=0.125, Interval=0.001, Tolerance=1e-8));
end SensorSampling;
