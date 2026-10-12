within fire_modelica_models.Tests;
model VoltagePropulsion "P1 analytic response, P0 equivalence, voltage sag and coast-down"
  Systems.Propulsion.SpeedDrivenRotor p0(tau=0.03,omegaMax=1000,omega_start=70,kQ=1.5e-7);
  Systems.Propulsion.VoltageDrivenRotor nominal(Kv=62.5,tauOmega=0.03,omega_start=70);
  Systems.Propulsion.VoltageDrivenRotor sag(Kv=62.5,tauOmega=0.03,omega_start=70);
  Systems.Propulsion.VoltageDrivenRotor stopped(Kv=62.5,tauOmega=0.03,omega_start=70);
  Real sagExpected(unit="rad/s");
equation
  p0.demand=0.4;
  p0.airVelocity_r=zeros(3);
  p0.omegaBody_r=zeros(3);
  nominal.escCommand=0.4;
  nominal.dcVoltage=16;
  nominal.airVelocity_r=zeros(3);
  nominal.omegaBody_r=zeros(3);
  sag.escCommand=0.4;
  sag.dcVoltage=if time<0.15 then 16 else 8;
  sag.airVelocity_r=zeros(3);
  sag.omegaBody_r=zeros(3);
  stopped.escCommand=1;
  stopped.dcVoltage=0;
  stopped.airVelocity_r=zeros(3);
  stopped.omegaBody_r=zeros(3);
  sagExpected=if time<0.15 then 400+(70-400)*exp(-time/0.03)
    else 200+(400+(70-400)*exp(-0.15/0.03)-200)*exp(-(time-0.15)/0.03);
  assert(abs(nominal.rotorSpeed-(400+(70-400)*exp(-time/0.03)))<1e-4,"P1 analytic response changed");
  assert(abs(nominal.rotorSpeed-p0.rotorSpeed)<1e-7,"P1 nominal speed differs from P0");
  assert(max(abs(nominal.force_r-p0.force_r))<1e-7 and max(abs(nominal.moment_r-p0.moment_r))<1e-7,
    "P1 nominal wrench differs from P0");
  assert(abs(sag.rotorSpeed-sagExpected)<1e-4,"Voltage sag response or independent state changed");
  assert(abs(stopped.rotorSpeed-70*exp(-time/0.03))<1e-4,"Zero voltage coast-down changed");
  annotation(experiment(StopTime=0.5,Tolerance=1e-10,Interval=0.001));
end VoltagePropulsion;
