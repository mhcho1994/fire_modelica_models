within fire_modelica_models.Examples;

model QuadGroundCycle "Drop, settle, open-loop takeoff, motor cutoff and landing in NED/FRD"
  Vehicles.Copter.MultirotorWithSensors vehicle(
    landingGear(enabled=true),p_start={0,0,-0.12});
  output Real altitude(unit="m") = -vehicle.body.p_w[3];
  output Real verticalVelocity(unit="m/s") = vehicle.body.v_w[3] "Down positive";
  output Real normalLoad(unit="N") = sum(vehicle.gear.normalForce);
  output Integer contactCount = sum(if vehicle.gear.contact[i] then 1 else 0 for i in 1:4);
  output Real acceleration_frd[3](each unit="m/s2") =
    vehicle.geometry.R_bImu*vehicle.measurements.acceleration;
equation
  vehicle.demand = {if time < 2 or time >= 2.7 then 0 else
    sqrt(1.3*vehicle.chassis.mass*9.80665/(vehicle.geometry.nRotors*vehicle.kT[i]))/vehicle.omegaMax[i]
    for i in 1:vehicle.geometry.nActuators};
  annotation(experiment(StopTime=6,Tolerance=1e-8,Interval=0.002),
    Documentation(info="<html><p>Unpowered feet start 2 cm above the plane. At 2 s a symmetric speed command requests 1.3 times weight in steady thrust. Motors switch off at 2.7 s, allowing a second landing. Plot altitude, verticalVelocity, normalLoad, contactCount and acceleration_frd[3]. This is an open-loop physics demonstration, not an ArduCopter flight controller.</p></html>"));
end QuadGroundCycle;
