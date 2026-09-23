within fire_modelica_models.Adapters.FastDyn;
model MultirotorValueAdapter "Flat value boundary for host integration; not a firmware driver"
  parameter Vehicles.Copter.Geometry geometry;
  parameter Real actuatorSamplePeriod=0.0025;
  parameter Real p_start[3]={0,0,1};
  input Real pwm_us[geometry.nActuators];
  PwmDemand commands(nChannels=geometry.nActuators,samplePeriod=actuatorSamplePeriod);
  Vehicles.Copter.MultirotorWithSensors vehicle(geometry=geometry,p_start=p_start);
  output Real acceleration_frd[3] "Specific force [m/s2]";
  output Real gyro_frd[3] "rad/s";
  output Real magneticField_frd[3] "T";
  output Real position_ned[3] "Local Cartesian; not latitude/longitude/altitude";
  output Real velocity_ned[3] "m/s";
  output Real pressure_Pa;
  output Real temperature_K;
  output Real altitude_m "Up positive";
  output Real climbRate_mps "Up positive";
  output Real sampleTimes[4] "IMU, magnetometer, GNSS, barometer [s]";
  output Real rotorSpeed[geometry.nRotors] "rad/s; all mapped rotor channels";
  output Interfaces.VehicleTruth truth "Separate ENU/FLU simulation truth";
protected
  constant Real fluToFrd[3,3]=diagonal({1,-1,-1});
  constant Real enuToNed[3,3]=[0,1,0;1,0,0;0,0,-1];
equation
  commands.pulseWidth_us=pwm_us;
  vehicle.demand=commands.demand;
  acceleration_frd=fluToFrd*geometry.R_bImu*vehicle.measurements.acceleration;
  gyro_frd=fluToFrd*geometry.R_bImu*vehicle.measurements.gyro;
  magneticField_frd=fluToFrd*geometry.R_bMag*vehicle.measurements.magneticField;
  position_ned=enuToNed*vehicle.measurements.position;
  velocity_ned=enuToNed*vehicle.measurements.velocity;
  pressure_Pa=vehicle.measurements.pressure;
  temperature_K=vehicle.measurements.temperature;
  altitude_m=vehicle.measurements.altitude;
  climbRate_mps=vehicle.measurements.climbRate;
  sampleTimes={vehicle.measurements.imuSampleTime,vehicle.measurements.magnetometerSampleTime,
    vehicle.measurements.gnssSampleTime,vehicle.measurements.barometerSampleTime};
  rotorSpeed=vehicle.rotors.rotorSpeed;
  truth=vehicle.truth;
end MultirotorValueAdapter;
