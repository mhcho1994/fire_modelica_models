within fire_modelica_models.Vehicles.Copter;
model VoltageDrivenMultirotorPlant "P1 voltage-driven N-rotor vehicle"
  extends PartialMultirotorPlant;
  parameter Real Kv[geometry.nRotors](each unit="rad/(s.V)") = fill(1000/16,geometry.nRotors);
  parameter Real tauOmega[geometry.nRotors](each unit="s") = fill(0.03,geometry.nRotors);
  parameter Real kT[geometry.nRotors] = fill(1e-5,geometry.nRotors);
  parameter Real kQ[geometry.nRotors] = fill(1.5e-7,geometry.nRotors);
  parameter Real rotorSpeed_start[geometry.nRotors] = zeros(geometry.nRotors);
  input Real escCommand[geometry.nActuators](each unit="1") "Unipolar voltage modulation command";
  input Modelica.Units.SI.Voltage dcVoltage "Shared DC bus; bound internally by the composer";
  replaceable model RotorUnit = Systems.Propulsion.VoltageDrivenRotor
    constrainedby Systems.Propulsion.PartialVoltageDrivenRotor;
  RotorUnit rotors[geometry.nRotors](Kv=Kv,tauOmega=tauOmega,kT=kT,kQ=kQ,
    spinSign=geometry.spinSign,omega_start=rotorSpeed_start);
equation
  for i in 1:geometry.nRotors loop
    rotors[i].escCommand = escCommand[geometry.actuatorIndex[i]];
    rotors[i].dcVoltage = dcVoltage;
    rotors[i].airVelocity_r = rotorAirVelocity_r[i,:];
    rotors[i].omegaBody_r = rotorOmegaBody_r[i,:];
    rotorForce_r[i,:] = rotors[i].force_r;
    rotorMoment_r[i,:] = rotors[i].moment_r;
  end for;
end VoltageDrivenMultirotorPlant;
