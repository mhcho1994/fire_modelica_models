within FIRE_Modelica.Systems.Sensing.GNSS.BaseClasses;

model IdealMeasurement "Default kinematic/environment observables without device dynamics"
  extends PartialMeasurement;
equation
  value[1:3] = position_w;
  value[4:6] = velocity_w;
end IdealMeasurement;
