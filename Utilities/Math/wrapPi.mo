within FIRE_Modelica.Utilities.Math;

function wrapPi
  input Real angle;
  output Real wrapped;
algorithm
  wrapped := mod(angle + Modelica.Constants.pi, 2 * Modelica.Constants.pi) - Modelica.Constants.pi;
end wrapPi;
