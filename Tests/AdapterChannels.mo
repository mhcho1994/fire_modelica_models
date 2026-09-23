within FIRE_Modelica.Tests;
model AdapterChannels "Eight independent PWM channels, clipping and value frame conversions"
  Adapters.FastDyn.MultirotorValueAdapter adapter(geometry=Data.Presets.OctoX(
      R_bImu=[0,-1,0;1,0,0;0,0,1],R_bMag=diagonal({1,-1,-1})),
    vehicle(gravity_w=zeros(3),p_start={1,2,10}));
  Adapters.Legacy.SensorBusAdapter legacy(R_bImu=adapter.geometry.R_bImu,R_bMag=adapter.geometry.R_bMag);
equation
  adapter.pwm_us={900,1100,1200,1300,1400,1500,1600,2100};
  legacy.truth=adapter.truth;
  legacy.measurements=adapter.vehicle.measurements;
  when time>=0.3 then
    assert(max(abs(adapter.commands.demand-{0,0.1,0.2,0.3,0.4,0.5,0.6,1}))<1e-12,"PWM channels must not be trimmed or truncated");
    assert(max(abs(adapter.rotorSpeed-1000*(1-exp(-time/0.03))*{0,0.1,0.2,0.3,0.4,0.5,0.6,1}))<0.01,"All eight motor channels must respond independently");
    assert(max(abs(adapter.position_ned-{adapter.vehicle.measurements.position[2],adapter.vehicle.measurements.position[1],-adapter.vehicle.measurements.position[3]}))<1e-12,"ENU to NED conversion failed");
    assert(max(abs(adapter.acceleration_frd-{-adapter.vehicle.measurements.acceleration[2],-adapter.vehicle.measurements.acceleration[1],-adapter.vehicle.measurements.acceleration[3]}))<1e-12,"Sensor to body FRD conversion failed");
    assert(max(abs(adapter.magneticField_frd-adapter.vehicle.measurements.magneticField))<1e-12,"Magnetometer mount conversion failed");
    assert(abs(legacy.sensor.ax+adapter.vehicle.measurements.acceleration[2])<1e-12 and
      abs(legacy.sensor.ay-adapter.vehicle.measurements.acceleration[1])<1e-12 and
      abs(legacy.sensor.az-adapter.vehicle.measurements.acceleration[3])<1e-12,"Legacy IMU body frame failed");
    assert(abs(legacy.sensor.my+adapter.vehicle.measurements.magneticField[2])<1e-12 and
      abs(legacy.sensor.mz+adapter.vehicle.measurements.magneticField[3])<1e-12,"Legacy magnetometer body frame failed");
  end when;
  annotation(experiment(StopTime=0.31,Tolerance=1e-8,Interval=0.001));
end AdapterChannels;
