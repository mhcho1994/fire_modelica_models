within FIRE_Modelica.Examples;

model QuadImuOnly "Direct IMU composition for a sensor experiment, without SensorSuite"
  Vehicles.Copter.MultirotorPlant vehicle(
    rotorSpeed_start={sqrt(vehicle.chassis.mass*9.80665/
      (vehicle.geometry.nRotors*vehicle.kT[i])) for i in 1:vehicle.geometry.nRotors});
  Systems.Sensing.IMU.Sensor imu(
    r_b=vehicle.geometry.imuPosition_C-vehicle.chassis.cg_C,
    R_bs=vehicle.geometry.R_bImu, samplePeriod=0.0025);
  output Real acceleration[3] "Specific force in IMU axes [m/s2]";
  output Real gyro[3] "Angular velocity in IMU axes [rad/s]";
  output Real sampleTime "Acquisition time [s]";
  output Interfaces.VehicleTruth truth;
equation
  vehicle.demand={vehicle.rotorSpeed_start[i]/vehicle.omegaMax[i]
    for i in 1:vehicle.geometry.nActuators};
  imu.R_wb=vehicle.body.R_wb;
  imu.a_w=vehicle.body.a_w;
  imu.gravity_w=vehicle.gravity_w;
  imu.omega_b=vehicle.body.omega_b;
  imu.alpha_b=vehicle.body.alpha_b;
  acceleration=imu.acceleration;
  gyro=imu.gyro;
  sampleTime=imu.sampleTime;
  truth=vehicle.truth;
  when time>=0.01 then
    assert(max(abs(acceleration-{0,0,9.80665}))<1e-8 and max(abs(gyro))<1e-8,
      "A directly composed IMU must preserve the hover measurement contract");
    assert(abs(sampleTime-time)<0.0025+1e-10,
      "A directly composed IMU must own its acquisition clock");
  end when;
  annotation(experiment(StopTime=1,Tolerance=1e-8,Interval=0.0025));
end QuadImuOnly;
