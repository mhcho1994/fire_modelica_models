within fire_modelica_models.Interfaces;

record SensorMeasurements "Sampled measurements with acquisition times, without attitude truth"
  Real acceleration[3](each unit="m/s2") "Specific force in IMU sensor frame";
  Real gyro[3](each unit="rad/s") "Angular velocity in IMU sensor frame";
  Real magneticField[3](each unit="T") "Field in magnetometer sensor frame";
  Real position[3](each unit="m") "GNSS antenna position in local ENU";
  Real velocity[3](each unit="m/s") "GNSS antenna velocity in local ENU";
  Real pressure(unit="Pa") "Ambient pressure with sensor bias";
  Real temperature(unit="K");
  Real altitude(unit="m") "Low-fidelity geometric ENU altitude with bias";
  Real climbRate(unit="m/s") "Low-fidelity geometric vertical velocity";
  Real imuSampleTime(unit="s");
  Real magnetometerSampleTime(unit="s");
  Real gnssSampleTime(unit="s");
  Real barometerSampleTime(unit="s");
end SensorMeasurements;
