within fire_modelica_models.Systems.Sensing.GNSS.BaseClasses;

model IdealMeasurement "Default kinematic/environment observables without device dynamics"
  extends PartialMeasurement;
equation
  value[1:3] = position_w;
  value[4:6] = velocity_w;
end IdealMeasurement;
