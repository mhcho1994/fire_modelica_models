within FIRE_Modelica.Systems.Sensing.Magnetometer.BaseClasses;

partial model PartialMeasurement "Continuous observable contract; no bias or sampling"
  parameter Real R_bs[3,3] = identity(3) "Sensor to body rotation";
  input Real R_wb[3,3];
  input Real magneticField_w[3](each unit="T");
  output Real value[3] "[magnetic field xyz (T)]";
end PartialMeasurement;
