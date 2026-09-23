within FIRE_Modelica.Systems.Propulsion;
model SpeedDrivenRotor "Empirical speed lag plus static blade aerodynamics"
  extends PartialSpeedDrivenRotor;
  ResponseModels.FirstOrderSpeed response(
    tau = tau, omegaMax = omegaMax, omega_start = omega_start);
  FIRE_Modelica.Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade blade(
    kT = kT, kQ = kQ, spinSign = spinSign);
equation
  response.demand = demand;
  blade.omega = response.omega;
  blade.airVelocity_r = airVelocity_r;
  blade.omegaBody_r = omegaBody_r;
  rotorSpeed = response.omega;
  force_r = blade.force_r;
  moment_r = blade.moment_r;
  annotation(Documentation(info = "<html><p>This empirical profile has no blade-load feedback into motor speed, no shaft-inertia reaction, no rotor gyroscopic torque, and no electrical power accounting. The output moment contains aerodynamic torque once; the vehicle adds the hub moment arm once.</p></html>"));
end SpeedDrivenRotor;
