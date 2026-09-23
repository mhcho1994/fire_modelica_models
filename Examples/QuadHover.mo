within FIRE_Modelica.Examples;
model QuadHover "Open-loop force balance with initialized rotor speed; no flight controller"
  parameter Vehicles.Copter.Geometry geometry=Vehicles.Copter.Presets.QuadX();
  Vehicles.Copter.MultirotorWithSensors vehicle(geometry=geometry,
    rotorSpeed_start={sqrt(vehicle.chassis.mass*9.80665/(geometry.nRotors*vehicle.kT[i])) for i in 1:geometry.nRotors});
equation
  vehicle.demand={sqrt(vehicle.chassis.mass*9.80665/(geometry.nRotors*vehicle.kT[i]))/vehicle.omegaMax[i] for i in 1:geometry.nActuators};
  annotation(experiment(StopTime=2,Tolerance=1e-7,Interval=0.005));
end QuadHover;
