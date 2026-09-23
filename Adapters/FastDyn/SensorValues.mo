within fire_modelica_models.Adapters.FastDyn;
model SensorValues "Stateless ENU/sensor-frame to NED/FRD value conversion"
  parameter Vehicles.Copter.Geometry geometry;
  input Interfaces.SensorMeasurements measurements;
  output Real acceleration_frd[3];
  output Real gyro_frd[3];
  output Real magneticField_frd[3] "Tesla";
  output Real position_ned[3];
  output Real velocity_ned[3];
protected
  constant Real fluToFrd[3,3]=diagonal({1,-1,-1});
  constant Real enuToNed[3,3]=[0,1,0;1,0,0;0,0,-1];
equation
  acceleration_frd=fluToFrd*geometry.R_bImu*measurements.acceleration;
  gyro_frd=fluToFrd*geometry.R_bImu*measurements.gyro;
  magneticField_frd=fluToFrd*geometry.R_bMag*measurements.magneticField;
  position_ned=enuToNed*measurements.position;
  velocity_ned=enuToNed*measurements.velocity;
end SensorValues;
