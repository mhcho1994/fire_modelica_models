within fire_modelica_models.Utilities.Math;

function clip
  input Real u;
  input Real uMin;
  input Real uMax;
  output Real y;
algorithm
  y := min(max(u, uMin), uMax);
end clip;
