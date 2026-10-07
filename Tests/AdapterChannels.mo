within fire_modelica_models.Tests;
model AdapterChannels "Eight independent PWM channels, clipping and value frame conversions"
  Adapters.FastDyn.MultirotorValueAdapter adapter(geometry=Vehicles.Copter.Presets.OctoX(
      R_bImu=[0,-1,0;1,0,0;0,0,1],R_bMag=diagonal({1,-1,-1})),
    landingGear(enabled=false),vehicle(gravity_w=zeros(3),p_start={1,2,10}));
equation
  adapter.pwm_us={900,1100,1200,1300,1400,1500,1600,2100};
  when time>=0.3 then
    assert(max(abs(adapter.commands.demand-{0,0.1,0.2,0.3,0.4,0.5,0.6,1}))<1e-12,"PWM channels must not be trimmed or truncated");
    assert(max(abs(adapter.rotorSpeed-1000*(1-exp(-time/0.03))*{0,0.1,0.2,0.3,0.4,0.5,0.6,1}))<0.01,"All eight motor channels must respond independently");
    assert(max(abs(adapter.position_ned-adapter.vehicle.measurements.position))<1e-12,"NED position must pass through without a second frame conversion");
    assert(max(abs(adapter.acceleration_frd-{-adapter.vehicle.measurements.acceleration[2],adapter.vehicle.measurements.acceleration[1],adapter.vehicle.measurements.acceleration[3]}))<1e-12,"Sensor to body FRD conversion failed");
    assert(max(abs(adapter.magneticField_frd-diagonal({1,-1,-1})*adapter.vehicle.measurements.magneticField))<1e-12,"Magnetometer mount conversion failed");
  end when;
  annotation(experiment(StopTime=0.31,Tolerance=1e-8,Interval=0.001));
end AdapterChannels;
