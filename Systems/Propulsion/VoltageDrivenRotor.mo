within fire_modelica_models.Systems.Propulsion;
model VoltageDrivenRotor "P1 ideal ESC, voltage-to-speed surrogate and quadratic blade"
  extends PartialVoltageDrivenRotor;
  Physical.Electrical.Drives.ESC.IdealVoltage esc;
  ResponseModels.VoltageToSpeed response(Kv=Kv, tauOmega=tauOmega, omega_start=omega_start);
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade blade(kT=kT, kQ=kQ, spinSign=spinSign);
equation
  esc.escCommand = escCommand;
  esc.dcVoltage = dcVoltage;
  response.equivalentVoltage = esc.equivalentVoltage;
  blade.omega = response.omega;
  blade.airVelocity_r = airVelocity_r;
  blade.omegaBody_r = omegaBody_r;
  rotorSpeed = response.omega;
  force_r = blade.force_r;
  moment_r = blade.moment_r;
  annotation(Documentation(info="<html><p>P1 owns one speed state in response. No current, power, shaft-load feedback, inertia reaction or gyroscopic moment. The aerodynamic hub moment is included once, as in P0.</p></html>"));
end VoltageDrivenRotor;
