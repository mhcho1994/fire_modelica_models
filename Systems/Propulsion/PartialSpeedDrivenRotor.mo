within FIRE_Modelica.Systems.Propulsion;
partial model PartialSpeedDrivenRotor "Common parameters for empirical speed-driven rotor profiles"
  extends PartialRotorUnit;
  parameter Modelica.Units.SI.Time tau = 0.02;
  parameter Real kT(unit = "N.s2") = 1e-5;
  parameter Real kQ(unit = "N.m.s2") = 1e-7;
  parameter Modelica.Units.SI.AngularVelocity omega_start = 0;
end PartialSpeedDrivenRotor;
