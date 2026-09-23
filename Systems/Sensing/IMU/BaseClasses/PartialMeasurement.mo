within FIRE_Modelica.Systems.Sensing.IMU.BaseClasses;

partial model PartialMeasurement "Continuous observable contract; no bias or sampling"
  parameter Real r_b[3](each unit="m") = zeros(3) "Sensor position relative to CG";
  parameter Real R_bs[3,3] = identity(3) "Sensor to body rotation";
  input Real R_wb[3,3];
  input Real a_w[3](each unit="m/s2");
  input Real gravity_w[3](each unit="m/s2");
  input Real omega_b[3](each unit="rad/s");
  input Real alpha_b[3](each unit="rad/s2");
  output Real value[6] "[specific force xyz (m/s2), angular velocity xyz (rad/s)]";
end PartialMeasurement;
