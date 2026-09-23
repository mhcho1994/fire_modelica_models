within FIRE_Modelica.Systems.Sensing.Magnetometer.BaseClasses;

model IdealMeasurement "Default kinematic/environment observables without device dynamics"
  extends PartialMeasurement;
equation
  value = transpose(R_bs) * transpose(R_wb) * magneticField_w;
end IdealMeasurement;
