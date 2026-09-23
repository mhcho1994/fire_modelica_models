within fire_modelica_models.Tests;

model SensorReconfiguration "Replace measurement and response components without changing sensor interfaces"
  model ScaledImu "Test-only measurement replacement with a known scale factor"
    extends Systems.Sensing.IMU.BaseClasses.PartialMeasurement;
    Systems.Sensing.IMU.BaseClasses.IdealMeasurement ideal(
      final r_b=r_b, final R_bs=R_bs);
  equation
    ideal.R_wb=R_wb;
    ideal.a_w=a_w;
    ideal.gravity_w=gravity_w;
    ideal.omega_b=omega_b;
    ideal.alpha_b=alpha_b;
    value[1]=2*ideal.value[1];
    value[2:6]=ideal.value[2:6];
  end ScaledImu;

  function rampResponse "Analytical first-order response to a unit ramp starting at zero"
    input Real t;
    input Real tau;
    output Real y;
  algorithm
    y := t-tau*(1-exp(-t/tau));
  end rampResponse;

  Systems.Sensing.SensorSuite suite(
    imuSamplePeriod=0.01, magnetometerSamplePeriod=0.02,
    gnssSamplePeriod=0.1, barometerSamplePeriod=0.05,
    accelBias={0.1,0,0}, pressureBias=10,
    imu(redeclare model Measurement = ScaledImu,
      redeclare model Response = Systems.Sensing.ResponseModels.FirstOrder(
        tau={0.02,0,0,0.03,0,0})),
    magnetometer(redeclare model Response = Systems.Sensing.ResponseModels.FirstOrder(
      tau={0.08,0,0})),
    gnss(redeclare model Response = Systems.Sensing.ResponseModels.FirstOrder(
      tau={0.04,0,0,0.05,0,0})),
    barometer(redeclare model Response = Systems.Sensing.ResponseModels.FirstOrder(
      tau={0.06,0,0.07,0})));
  Systems.Sensing.IMU.Sensor direct(
    samplePeriod=0.01,
    redeclare model Response = Systems.Sensing.ResponseModels.FirstOrder(
      tau={0.02,0,0,0,0,0}));
equation
  suite.p_w={time,2*time,3*time};
  suite.v_w={1,2,3};
  suite.R_wb=identity(3);
  suite.a_w={time,2*time,0};
  suite.omega_b={time,0,0};
  suite.alpha_b={1,0,0};
  suite.gravity_w={0,0,-9.81};
  suite.magneticField_w={1e-5+1e-6*time,2e-5,3e-5};
  suite.pressure=101325+100*time;
  suite.temperature=288.15+time;
  direct.R_wb=suite.R_wb;
  direct.a_w=suite.a_w;
  direct.gravity_w=suite.gravity_w;
  direct.omega_b=suite.omega_b;
  direct.alpha_b=suite.alpha_b;

  // Check between acquisition events, including the first held sample.
  when sample(0.005,0.01) then
    assert(abs(suite.imu.sampleTime-0.01*floor(time/0.01))<1e-10 and
      abs(suite.gnss.sampleTime-0.1*floor(time/0.1))<1e-10 and
      abs(suite.magnetometer.sampleTime-0.02*floor(time/0.02))<1e-10 and
      abs(suite.barometer.sampleTime-0.05*floor(time/0.05))<1e-10,
      "Response replacement must preserve independent acquisition clocks");
    assert(abs(suite.imu.acceleration[1] -
        (2*rampResponse(suite.imu.sampleTime,0.02)+0.1))<1e-7 and
      abs(suite.imu.acceleration[2]-2*suite.imu.sampleTime)<1e-10 and
      abs(suite.imu.acceleration[3]-9.81)<1e-10 and
      abs(suite.imu.gyro[1]-rampResponse(suite.imu.sampleTime,0.03))<1e-7,
      "Measurement replacement, response channel order, bypass or bias ordering changed");
    assert(abs(direct.acceleration[1]-rampResponse(direct.sampleTime,0.02))<1e-7,
      "A directly composed sensor must support the same response replacement");
    assert(abs(suite.gnss.position[1]-rampResponse(suite.gnss.sampleTime,0.04))<1e-7 and
      abs(suite.gnss.position[2]-2*suite.gnss.sampleTime)<1e-10 and
      max(abs(suite.gnss.velocity-{1,2,3}))<1e-10,
      "GNSS response must initialize at its input and hold filtered acquisitions");
    assert(abs(suite.magnetometer.magneticField[1] -
      (1e-5+1e-6*rampResponse(suite.magnetometer.sampleTime,0.08)))<1e-12,
      "Magnetometer response must preserve Tesla units and acquisition timing");
    assert(abs(suite.barometer.pressure -
        (101335+100*rampResponse(suite.barometer.sampleTime,0.06)))<1e-5 and
      abs(suite.barometer.temperature-(288.15+suite.barometer.sampleTime))<1e-9 and
      abs(suite.barometer.altitude-3*rampResponse(suite.barometer.sampleTime,0.07))<1e-7 and
      abs(suite.barometer.climbRate-3)<1e-10,
      "Barometer pressure, temperature and geometric channel semantics changed");
  end when;
  annotation(experiment(StopTime=0.225,Interval=0.001,Tolerance=1e-10));
end SensorReconfiguration;
