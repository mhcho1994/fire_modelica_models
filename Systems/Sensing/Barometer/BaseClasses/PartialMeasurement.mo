within fire_modelica_models.Systems.Sensing.Barometer.BaseClasses;

partial model PartialMeasurement "Continuous observable contract; no bias or sampling"
  input Real ambientPressure(unit="Pa");
  input Real ambientTemperature(unit="K");
  input Real altitude_w(unit="m");
  input Real climbRate_w(unit="m/s");
  output Real value[4] "[ambient pressure (Pa), temperature (K), geometric altitude (m), climb rate (m/s)]";
end PartialMeasurement;
