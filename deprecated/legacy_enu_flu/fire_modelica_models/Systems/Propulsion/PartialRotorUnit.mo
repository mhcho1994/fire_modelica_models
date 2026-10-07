within fire_modelica_models.Systems.Propulsion;
partial model PartialRotorUnit "Rotor hub interface before mounting and CG moment translation"
  parameter Modelica.Units.SI.AngularVelocity omegaMax = 1000;
  parameter Integer spinSign = 1 "Right-hand rotation about positive thrust axis";
  input Real demand "Normalized speed demand, limited to [0,1] by the speed profile";
  input Modelica.Units.SI.Velocity airVelocity_r[3]
    "Hub velocity relative to air in rotor coordinates";
  input Modelica.Units.SI.AngularVelocity omegaBody_r[3];
  output Modelica.Units.SI.Force force_r[3];
  output Modelica.Units.SI.Torque moment_r[3] "Hub moment in rotor coordinates";
  output Modelica.Units.SI.AngularVelocity rotorSpeed;
initial equation
  assert(omegaMax > 0, "Rotor maximum speed must be positive");
  assert(abs(spinSign) == 1, "Rotor spinSign must be +1 or -1");
end PartialRotorUnit;
