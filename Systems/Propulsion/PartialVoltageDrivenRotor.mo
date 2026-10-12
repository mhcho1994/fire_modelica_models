within fire_modelica_models.Systems.Propulsion;
partial model PartialVoltageDrivenRotor "P1 ESC command and voltage boundary; rotor-local hub wrench"
  parameter Real Kv(unit="rad/(s.V)") = 1000/16;
  parameter Modelica.Units.SI.Time tauOmega = 0.03;
  parameter Real kT(unit="N.s2") = 1e-5;
  parameter Real kQ(unit="N.m.s2") = 1.5e-7;
  parameter Modelica.Units.SI.AngularVelocity omega_start = 0;
  parameter Integer spinSign = 1;
  input Real escCommand(unit="1") "Unipolar voltage modulation command";
  input Modelica.Units.SI.Voltage dcVoltage;
  input Modelica.Units.SI.Velocity airVelocity_r[3];
  input Modelica.Units.SI.AngularVelocity omegaBody_r[3];
  output Modelica.Units.SI.Force force_r[3];
  output Modelica.Units.SI.Torque moment_r[3] "Aerodynamic moment at rotor hub";
  output Modelica.Units.SI.AngularVelocity rotorSpeed;
initial equation
  assert(abs(spinSign) == 1, "Rotor spinSign must be +1 or -1");
end PartialVoltageDrivenRotor;
