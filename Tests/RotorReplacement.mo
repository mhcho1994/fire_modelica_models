within FIRE_Modelica.Tests;
model RotorReplacement "Replaceable rotor array preserves common speed-profile parameters"
  model ReducedThrustRotor
    extends Systems.Propulsion.SpeedDrivenRotor(blade(kT=0.5*kT));
  end ReducedThrustRotor;
  Vehicles.Copter.MultirotorPlant vehicle(
    redeclare model RotorUnit=ReducedThrustRotor,
    geometry(nLegs=0),rotorSpeed_start=fill(100,4),gravity_w=zeros(3));
equation
  vehicle.demand=fill(0.1,4);
  assert(abs(vehicle.force_b[3]-0.2)<1e-10,"Redeclared rotor array did not use replacement blade coefficient");
  annotation(experiment(StopTime=0.05,Tolerance=1e-9,Interval=0.001));
end RotorReplacement;
