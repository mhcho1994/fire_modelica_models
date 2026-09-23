within FIRE_Modelica.Adapters.FastDyn;

model MultirotorFmu "Flat FMI boundary with sampled measurements and separate continuous truth"
  parameter FIRE_Modelica.Vehicles.Copter.Geometry geometry;
  parameter Real p_start[3]={0,0,1};
  parameter Real actuatorSamplePeriod=0.0025;
  input Real pwm_us[geometry.nActuators];
  FIRE_Modelica.Adapters.FastDyn.MultirotorValueAdapter adapter(
    geometry=geometry, p_start=p_start, actuatorSamplePeriod=actuatorSamplePeriod);
  output Real acceleration_frd[3] "Specific force [m/s2]";
  output Real gyro_frd[3] "rad/s";
  output Real magneticField_frd[3] "T";
  output Real position_ned[3] "Sampled local Cartesian antenna position [m]";
  output Real velocity_ned[3] "Sampled antenna velocity [m/s]";
  output Real pressure_Pa;
  output Real temperature_K;
  output Real altitude_m "Sampled up-positive geometric altitude";
  output Real climbRate_mps;
  output Real sampleTimes[4] "IMU, magnetometer, GNSS, barometer acquisition times [s]";
  output Real rotorSpeed[geometry.nRotors] "rad/s";
  output Real truthPosition_w[3] "Continuous CG truth in ENU [m]";
  output Real truthVelocity_w[3] "Continuous CG truth in ENU [m/s]";
  output Real truthQuaternion_wb[4] "Continuous scalar-first Hamilton body-to-world attitude";
equation
  adapter.pwm_us = pwm_us;
  acceleration_frd = adapter.acceleration_frd;
  gyro_frd = adapter.gyro_frd;
  magneticField_frd = adapter.magneticField_frd;
  position_ned = adapter.position_ned;
  velocity_ned = adapter.velocity_ned;
  pressure_Pa = adapter.pressure_Pa;
  temperature_K = adapter.temperature_K;
  altitude_m = adapter.altitude_m;
  climbRate_mps = adapter.climbRate_mps;
  sampleTimes = adapter.sampleTimes;
  rotorSpeed = adapter.rotorSpeed;
  truthPosition_w = adapter.truth.p_w;
  truthVelocity_w = adapter.truth.v_w;
  truthQuaternion_wb = adapter.truth.q_wb;
end MultirotorFmu;
