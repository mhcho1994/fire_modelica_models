within FIRE_Modelica.Examples;
model FixedPayload "Offset rigid payload changes mass, CG, inertia and every mount"
  Vehicles.Copter.MultirotorWithSensors vehicle(useAssembledMass=true,
    nPayloads=1,payloads={Physical.Mechanical.Chassis.Payloads.FixedPayload(
      mass=0.3,r_C={0.05,0,-0.03},inertia=diagonal({0.001,0.001,0.001}))},
    p_start={0,0,0.4});
equation
  vehicle.demand=zeros(vehicle.geometry.nActuators);
  annotation(experiment(StopTime=3,Tolerance=1e-7,Interval=0.001));
end FixedPayload;
