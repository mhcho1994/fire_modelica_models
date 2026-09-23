within FIRE_Modelica.Adapters.Legacy;
model SensorBusAdapter "Explicit mixed truth/measurement compatibility view"
  parameter Real R_bImu[3,3]=identity(3) "IMU sensor axes to body FLU";
  parameter Real R_bMag[3,3]=identity(3) "Magnetometer sensor axes to body FLU";
  input Interfaces.VehicleTruth truth;
  input Interfaces.SensorMeasurements measurements;
  Interfaces.SensorBus sensor;
protected
  Modelica.Blocks.Interfaces.RealOutput values[25]
    "Typed sources make all compatibility expandable-bus members present";
equation
  connect(sensor.x,values[1]);
  connect(sensor.y,values[2]);
  connect(sensor.z,values[3]);
  connect(sensor.vx,values[4]);
  connect(sensor.vy,values[5]);
  connect(sensor.vz,values[6]);
  connect(sensor.u,values[7]);
  connect(sensor.v,values[8]);
  connect(sensor.w,values[9]);
  connect(sensor.ax,values[10]);
  connect(sensor.ay,values[11]);
  connect(sensor.az,values[12]);
  connect(sensor.p,values[13]);
  connect(sensor.q,values[14]);
  connect(sensor.r,values[15]);
  connect(sensor.mx,values[16]);
  connect(sensor.my,values[17]);
  connect(sensor.mz,values[18]);
  connect(sensor.phi,values[19]);
  connect(sensor.theta,values[20]);
  connect(sensor.psi,values[21]);
  connect(sensor.pressure,values[22]);
  connect(sensor.temperature,values[23]);
  connect(sensor.baroAltitude,values[24]);
  connect(sensor.baroClimbRate,values[25]);
  values[1:3]=measurements.position;
  values[4:6]=measurements.velocity;
  values[7:9]=truth.v_b;
  values[10:12]=R_bImu*measurements.acceleration;
  values[13:15]=R_bImu*measurements.gyro;
  values[16:18]=R_bMag*measurements.magneticField;
  values[19:21]={atan2(truth.R_wb[3,2],truth.R_wb[3,3]),
    asin(min(1,max(-1,-truth.R_wb[3,1]))),atan2(truth.R_wb[2,1],truth.R_wb[1,1])};
  values[22:25]={measurements.pressure,measurements.temperature,measurements.altitude,measurements.climbRate};
end SensorBusAdapter;
