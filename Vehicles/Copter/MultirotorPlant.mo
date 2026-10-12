within fire_modelica_models.Vehicles.Copter;
model MultirotorPlant "P0 speed-driven N-rotor vehicle"
  extends PartialMultirotorPlant;
  parameter Real omegaMax[geometry.nRotors] = fill(1000,geometry.nRotors) "rad/s";
  parameter Real motorTau[geometry.nRotors] = fill(0.03,geometry.nRotors) "s";
  parameter Real kT[geometry.nRotors] = fill(1e-5,geometry.nRotors) "N/(rad/s)^2";
  parameter Real kQ[geometry.nRotors] = fill(1.5e-7,geometry.nRotors) "N.m/(rad/s)^2";
  parameter Real rotorSpeed_start[geometry.nRotors] = zeros(geometry.nRotors);
  input Real demand[geometry.nActuators](each unit="1") "Normalized rotor speed command [0,1], mapped by actuatorIndex";
  replaceable model RotorUnit = Systems.Propulsion.SpeedDrivenRotor
    constrainedby Systems.Propulsion.PartialSpeedDrivenRotor;
  RotorUnit rotors[geometry.nRotors](omegaMax=omegaMax,spinSign=geometry.spinSign,
    tau=motorTau,kT=kT,kQ=kQ,omega_start=rotorSpeed_start);
equation
  for i in 1:geometry.nRotors loop
    rotors[i].demand = demand[geometry.actuatorIndex[i]];
    rotors[i].airVelocity_r = rotorAirVelocity_r[i,:];
    rotors[i].omegaBody_r = rotorOmegaBody_r[i,:];
    rotorForce_r[i,:] = rotors[i].force_r;
    rotorMoment_r[i,:] = rotors[i].moment_r;
  end for;
end MultirotorPlant;
