within fire_modelica_models.Interfaces;

record VehicleTruth "Continuous rigid-body truth; never a sensor measurement"
  Real p_w[3](each unit="m") "CG position in ENU world";
  Real v_w[3](each unit="m/s") "CG velocity in ENU world";
  Real v_b[3](each unit="m/s") "CG velocity resolved in FLU body";
  Real q_wb[4] "Scalar-first Hamilton quaternion, body to world";
  Real R_wb[3,3] "Rotation from body to world";
  Real omega_b[3](each unit="rad/s") "Body angular velocity";
  Real a_w[3](each unit="m/s2") "CG inertial acceleration in world";
  Real alpha_b[3](each unit="rad/s2") "Body angular acceleration";
end VehicleTruth;
