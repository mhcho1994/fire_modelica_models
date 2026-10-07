within fire_modelica_models.Examples;
model QuadDrop "Drop onto four independent compliant legs with sampled sensors"
  Vehicles.Copter.MultirotorWithSensors vehicle(p_start={0,0,0.5});
equation
  vehicle.demand=zeros(vehicle.geometry.nActuators);
  annotation(experiment(StopTime=3,Tolerance=1e-7,Interval=0.001));
end QuadDrop;
