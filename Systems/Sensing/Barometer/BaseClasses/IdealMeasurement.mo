within fire_modelica_models.Systems.Sensing.Barometer.BaseClasses;

model IdealMeasurement "Default kinematic/environment observables without device dynamics"
  extends PartialMeasurement;
equation
  value = {ambientPressure, ambientTemperature, altitude_w, climbRate_w};
end IdealMeasurement;
