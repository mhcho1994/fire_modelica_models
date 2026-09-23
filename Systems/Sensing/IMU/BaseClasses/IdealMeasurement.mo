within fire_modelica_models.Systems.Sensing.IMU.BaseClasses;

model IdealMeasurement "Default kinematic/environment observables without device dynamics"
  extends PartialMeasurement;
protected
  Real specificForce_b[3];
equation
  specificForce_b = transpose(R_wb) * (a_w - gravity_w)
    + cross(alpha_b, r_b) + cross(omega_b, cross(omega_b, r_b));
  value[1:3] = transpose(R_bs) * specificForce_b;
  value[4:6] = transpose(R_bs) * omega_b;
end IdealMeasurement;
