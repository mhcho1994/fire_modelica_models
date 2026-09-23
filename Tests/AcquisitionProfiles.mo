within fire_modelica_models.Tests;
model AcquisitionProfiles "Continuous outputs and legacy sample/hold share measurement dynamics"
  Systems.Sensing.SensorSuite sensors[2](sampled={false,true},
    each imuSamplePeriod=0.01, each magnetometerSamplePeriod=0.01,
    each gnssSamplePeriod=0.01, each barometerSamplePeriod=0.01);
  Adapters.FastDyn.PwmDemand commands[2](sampled={false,true},
    each nChannels=6, each samplePeriod=0.01);
equation
  for i in 1:2 loop
    sensors[i].p_w={time,0,0};
    sensors[i].v_w={1,0,0};
    sensors[i].R_wb=identity(3);
    sensors[i].a_w={time,0,0};
    sensors[i].omega_b={time,0,0};
    sensors[i].alpha_b=zeros(3);
    sensors[i].gravity_w={0,0,-9.80665};
    sensors[i].magneticField_w={time,0,0};
    sensors[i].pressure=101325+time;
    sensors[i].temperature=288.15;
    commands[i].pulseWidth_us=fill(1000+10000*time,6);
  end for;
  assert(noEvent(abs(sensors[1].measurements.acceleration[1]-time)<1e-8),"Continuous IMU must track time");
  assert(noEvent(abs(sensors[1].measurements.magneticField[1]-time)<1e-8),"Continuous magnetometer must track time");
  assert(noEvent(abs(sensors[1].measurements.position[1]-time)<1e-8),"Continuous GNSS must track time");
  assert(noEvent(abs(sensors[1].measurements.pressure-101325-time)<1e-8),"Continuous barometer must track time");
  assert(noEvent(abs(sensors[2].measurements.acceleration[1]-sensors[2].measurements.imuSampleTime)<1e-8),"Sampled IMU must hold its last value");
  assert(noEvent(abs(sensors[2].measurements.magneticField[1]-sensors[2].measurements.magnetometerSampleTime)<1e-8),"Sampled magnetometer must hold its last value");
  assert(noEvent(abs(sensors[2].measurements.position[1]-sensors[2].measurements.gnssSampleTime)<1e-8),"Sampled GNSS must hold its last value");
  assert(noEvent(abs(sensors[2].measurements.pressure-101325-sensors[2].measurements.barometerSampleTime)<1e-8),"Sampled barometer must hold its last value");
end AcquisitionProfiles;
