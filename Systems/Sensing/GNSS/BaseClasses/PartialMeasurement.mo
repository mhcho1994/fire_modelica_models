within fire_modelica_models.Systems.Sensing.GNSS.BaseClasses;

partial model PartialMeasurement "Continuous observable contract; no bias or sampling"
  input Real position_w[3](each unit="m");
  input Real velocity_w[3](each unit="m/s");
  output Real value[6] "[antenna position xyz (m), antenna velocity xyz (m/s)]";
end PartialMeasurement;
