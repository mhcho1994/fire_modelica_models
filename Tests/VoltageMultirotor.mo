within fire_modelica_models.Tests;
model VoltageMultirotor "P0/P1 vehicle equivalence with remapped channels and shared contact"
  parameter Vehicles.Copter.Geometry geometry=Vehicles.Copter.Presets.QuadX(
    actuatorIndex={3,2,4,1},spinSign={-1,1,-1,1});
  Vehicles.Copter.MultirotorWithSensors p0(geometry=geometry,p_start={0,0,-0.12});
  Vehicles.Copter.VoltageDrivenMultirotorWithSensors p1(geometry=geometry,p_start={0,0,-0.12});
equation
  p0.demand={0.1,0.2,0.3,0.4};
  p1.escCommand={0.1,0.2,0.3,0.4};
  p1.dcVoltage=16;
  assert(max(abs(p0.rotors.rotorSpeed-p1.rotors.rotorSpeed))<1e-6,"Vehicle rotor mapping changed");
  assert(max(abs(p0.force_b-p1.force_b))<1e-4,"P1 total force changed at nominal voltage");
  assert(max(abs(p0.moment_b-p1.moment_b))<1e-4,"P1 hub/CG moment changed");
  assert(max(abs(p0.truth.p_w-p1.truth.p_w))<1e-6,"P1 body motion changed");
  assert(max(abs(p0.measurements.acceleration-p1.measurements.acceleration))<1e-4,"P1 sensing changed");
  annotation(experiment(StopTime=0.2,Tolerance=1e-10,Interval=0.001));
end VoltageMultirotor;
