within FIRE_Modelica.Systems.Sensing.ResponseModels;

partial model PartialResponse "Preserve the count, ordering and SI units of measurement channels"
  parameter Integer n(min=1) = 1;
  input Real u[n] "Continuous observables in the sensor family's channel order";
  output Real y[n] "Continuous response, with the same units and frame as u";
end PartialResponse;
