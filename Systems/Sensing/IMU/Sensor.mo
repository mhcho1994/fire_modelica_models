within fire_modelica_models.Systems.Sensing.IMU;

model Sensor "Continuous specific force and rate for a sensor rigidly mounted on the body"
  parameter Real r_b[3](each unit="m") = zeros(3) "Sensor position relative to CG";
  parameter Real R_bs[3,3] = identity(3) "Body-to-sensor relative attitude; v_b = R_bs*v_s";
  parameter Real accelBias[3](each unit="m/s2") = zeros(3);
  parameter Real gyroBias[3](each unit="rad/s") = zeros(3);
  input Real R_wb[3,3];
  input Real a_w[3](each unit="m/s2");
  input Real gravity_w[3](each unit="m/s2");
  input Real omega_b[3](each unit="rad/s");
  input Real alpha_b[3](each unit="rad/s2");
  output Real acceleration[3](each unit="m/s2");
  output Real gyro[3](each unit="rad/s");

protected
  Real specificForce_b[3](each unit="m/s2");
initial equation
  assert(max(abs(transpose(R_bs)*R_bs-identity(3)))<1e-9,"Sensor mount must be a proper rotation");
    // and abs(Modelica.Math.Matrices.det(R_bs)-1)<1e-9
equation
  specificForce_b = transpose(R_wb)*(a_w-gravity_w)
    + cross(alpha_b,r_b) + cross(omega_b,cross(omega_b,r_b));
  acceleration = transpose(R_bs)*specificForce_b + accelBias;
  gyro = transpose(R_bs)*omega_b + gyroBias;
end Sensor;
