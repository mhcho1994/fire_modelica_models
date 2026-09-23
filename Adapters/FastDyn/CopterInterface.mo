within fire_modelica_models.Adapters.FastDyn;
partial model CopterInterface "FastDyn FMI value contract; the composer supplies one plant"
  parameter Vehicles.Copter.Geometry geometry;
  parameter Boolean sampledActuators = true;
  parameter Real actuatorSamplePeriod = 0.0025;
  parameter Real pwm_min = 1100;
  parameter Real pwm_max = 1900;
  parameter Real lat0 = 40.414929;
  parameter Real lon0 = -86.932387;
  parameter Real ground_alt_wgs84 = 149;
  parameter Real earth_radius_m = 6378137;
  input Real pwm[geometry.nActuators](each start=1000);
  output Real accel[3] "Body FRD specific force [m/s2]";
  output Real gyro[3] "Body FRD [rad/s]";
  output Real mag[3] "Body FRD [Gauss]";
  output Real gps[3] "Latitude/longitude [deg], WGS84 altitude [m]";
  output Real vel_ned[3] "NED [m/s]";
  output Real yaw_deg "Heading clockwise from north [deg]";
  output Real baro_altitude_m;
  output Real baro_pressure_pa;
  output Real baro_temperature_c;
  output Real baro_climb_rate_mps;
  output Real rotorSpeed[geometry.nRotors];
  output Interfaces.VehicleTruth truth;
  PwmDemand commands(nChannels=geometry.nActuators, sampled=sampledActuators,
    samplePeriod=actuatorSamplePeriod, pwmMin=fill(pwm_min,geometry.nActuators),
    pwmMax=fill(pwm_max,geometry.nActuators));
  SensorValues values(geometry=geometry);
protected
  Real ambientPressure;
  Real ambientTemperature;
  Real altitudeAbsolute;
equation
  commands.pulseWidth_us=pwm;
  accel=values.acceleration_frd;
  gyro=values.gyro_frd;
  mag=10000*values.magneticField_frd;
  gps[1]=lat0+values.position_ned[1]/earth_radius_m*180/Modelica.Constants.pi;
  gps[2]=lon0+values.position_ned[2]/(earth_radius_m*cos(lat0*Modelica.Constants.pi/180))*180/Modelica.Constants.pi;
  gps[3]=ground_alt_wgs84-values.position_ned[3];
  vel_ned=values.velocity_ned;
  yaw_deg=atan2(truth.R_wb[1,1],truth.R_wb[2,1])*180/Modelica.Constants.pi;
  baro_altitude_m=values.measurements.altitude;
  baro_pressure_pa=values.measurements.pressure;
  baro_temperature_c=values.measurements.temperature-273.15;
  baro_climb_rate_mps=values.measurements.climbRate;
  altitudeAbsolute=ground_alt_wgs84+truth.p_w[3];
  ambientTemperature=288.15-0.0065*altitudeAbsolute;
  ambientPressure=101325*(1-2.25577e-5*altitudeAbsolute)^5.25588;
end CopterInterface;
