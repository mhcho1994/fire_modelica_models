within fire_modelica_models.Tests;
model PropulsionStep "Analytical speed step, opposite yaw torques, and demand limits"
  fire_modelica_models.Systems.Propulsion.SpeedDrivenRotor ccw(
    omegaMax = 100, tau = 0.05, kT = 1e-3, kQ = 2e-5, spinSign = 1);
  fire_modelica_models.Systems.Propulsion.SpeedDrivenRotor cw(
    omegaMax = 100, tau = 0.05, kT = 1e-3, kQ = 2e-5, spinSign = -1);
  fire_modelica_models.Systems.Propulsion.ResponseModels.FirstOrderSpeed saturated(
    omegaMax = 100, tau = 0.05);
  fire_modelica_models.Systems.Propulsion.ResponseModels.FirstOrderSpeed stopped(
    omegaMax = 100, tau = 0.05);
  Real expectedSpeed;
equation
  ccw.demand = 0.5;
  cw.demand = 0.5;
  ccw.airVelocity_r = zeros(3);
  cw.airVelocity_r = zeros(3);
  ccw.omegaBody_r = zeros(3);
  cw.omegaBody_r = zeros(3);
  saturated.demand = 2;
  stopped.demand = -1;
  expectedSpeed = 50 * (1 - exp(-time / 0.05));
  assert(abs(ccw.rotorSpeed - expectedSpeed) < 1e-5 and
    abs(cw.rotorSpeed - expectedSpeed) < 1e-5, "Rotor speed step response is incorrect");
  assert(abs(ccw.force_r[3] - 1e-3 * expectedSpeed ^ 2) < 1e-5,
    "Thrust must follow actual rotor speed, not command");
  assert(abs(ccw.moment_r[3] + 2e-5 * expectedSpeed ^ 2) < 1e-6 and
    abs(cw.moment_r[3] - 2e-5 * expectedSpeed ^ 2) < 1e-6,
    "Aerodynamic yaw torque must oppose spin direction");
  assert(max(abs(ccw.moment_r + cw.moment_r)) < 1e-9,
    "Oppositely spinning identical rotors must cancel aerodynamic yaw torque");
  assert(abs(saturated.omega - 2 * expectedSpeed) < 2e-5 and abs(stopped.omega) < 1e-10,
    "Normalized speed demand must be limited to [0,1]");
  annotation(experiment(StopTime = 0.5, Tolerance = 1e-10, Interval = 0.005));
end PropulsionStep;
