within fire_modelica_models.Adapters.FastDyn;

model MultirotorFmu "Flat FMI boundary with continuous measurements and separate continuous truth"
  parameter fire_modelica_models.Vehicles.Copter.Geometry geometry;
  parameter fire_modelica_models.Physical.Mechanical.Chassis.LandingGear.Parameters landingGear;
  parameter Real p_start[3]={0,0,-1};
  input Real pwm_us[geometry.nActuators];
  fire_modelica_models.Adapters.FastDyn.MultirotorValueAdapter adapter(
    geometry=geometry, p_start=p_start, landingGear=landingGear);
  output Real acceleration_frd[3] "Specific force [m/s2]";
  output Real gyro_frd[3] "rad/s";
  output Real magneticField_frd[3] "T";
  output Real position_ned[3] "Continuous local Cartesian antenna position [m]";
  output Real velocity_ned[3] "Continuous antenna velocity [m/s]";
  output Real pressure_Pa;
  output Real temperature_K;
  output Real altitude_m "Continuous up-positive geometric altitude";
  output Real climbRate_mps;
  output Real rotorSpeed[geometry.nRotors] "rad/s";
  output Real truthPosition_w[3] "Continuous CG truth in NED [m]";
  output Real truthVelocity_w[3] "Continuous CG truth in NED [m/s]";
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
  rotorSpeed = adapter.rotorSpeed;
  truthPosition_w = adapter.truth.p_w;
  truthVelocity_w = adapter.truth.v_w;
  truthQuaternion_wb = adapter.truth.q_wb;
end MultirotorFmu;
