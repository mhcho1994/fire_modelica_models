within fire_modelica_models.Examples;
model QuadVoltageSag "P1 supply drop reduces speed and thrust at a fixed ESC command"
  parameter Real hoverCommand=sqrt(1.5*9.80665/(4*1e-5))/1000;
  Vehicles.Copter.VoltageDrivenMultirotorWithSensors vehicle(
    geometry=Vehicles.Copter.Presets.QuadX(),landingGear(enabled=false),
    rotorSpeed_start=fill(1000*hoverCommand,4));
  output Real dcVoltage(unit="V");
  output Real altitude(unit="m");
equation
  dcVoltage=if time<1 then 16 else 12;
  vehicle.dcVoltage=dcVoltage;
  vehicle.escCommand=fill(hoverCommand,4);
  altitude=-vehicle.truth.p_w[3];
  annotation(experiment(StopTime=2,Tolerance=1e-8,Interval=0.002));
end QuadVoltageSag;
