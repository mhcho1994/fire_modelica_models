within FIRE_Modelica.Examples;

model QuadImuResponse "Direct IMU assembly with first-order acceleration and gyro responses"
  extends QuadImuOnly(
    imu(redeclare model Response = Systems.Sensing.ResponseModels.FirstOrder(
      tau={0.01,0.01,0.01,0.005,0.005,0.005})));
  annotation(experiment(StopTime=1,Tolerance=1e-8,Interval=0.0025));
end QuadImuResponse;
