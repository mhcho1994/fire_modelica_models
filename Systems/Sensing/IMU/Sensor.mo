within FIRE_Modelica.Systems.Sensing.IMU;

model Sensor "Sampled specific force and rate for a sensor rigidly mounted on the body"
  parameter Real samplePeriod(unit="s") = 0.0025;
  parameter Real r_b[3](each unit="m") = zeros(3) "Sensor position relative to CG";
  parameter Real R_bs[3,3] = identity(3) "Sensor to body rotation";
  parameter Real accelBias[3](each unit="m/s2") = zeros(3);
  parameter Real gyroBias[3](each unit="rad/s") = zeros(3);
  input Real R_wb[3,3];
  input Real a_w[3](each unit="m/s2");
  input Real gravity_w[3](each unit="m/s2");
  input Real omega_b[3](each unit="rad/s");
  input Real alpha_b[3](each unit="rad/s2");
  discrete output Real acceleration[3](each unit="m/s2", each start=0, each fixed=true);
  discrete output Real gyro[3](each unit="rad/s", each start=0, each fixed=true);
  discrete output Real sampleTime(unit="s", start=0, fixed=true);

  replaceable model Measurement = BaseClasses.IdealMeasurement
    constrainedby BaseClasses.PartialMeasurement
    "Continuous measurement model in the family channel order"
    annotation(choicesAllMatching=true);
  replaceable model Response = FIRE_Modelica.Systems.Sensing.ResponseModels.Ideal
    constrainedby FIRE_Modelica.Systems.Sensing.ResponseModels.PartialResponse
    "Continuous response before bias and acquisition"
    annotation(choicesAllMatching=true);
  Measurement measurement(final r_b=r_b, final R_bs=R_bs);
  Response response(final n=6);
initial equation
  assert(samplePeriod > 0, "IMU samplePeriod must be positive");
equation
  measurement.R_wb = R_wb;
  measurement.a_w = a_w;
  measurement.gravity_w = gravity_w;
  measurement.omega_b = omega_b;
  measurement.alpha_b = alpha_b;
  response.u = measurement.value;
  when sample(0, samplePeriod) then
    acceleration = response.y[1:3] + accelBias;
    gyro = response.y[4:6] + gyroBias;
    sampleTime = time;
  end when;
end Sensor;
