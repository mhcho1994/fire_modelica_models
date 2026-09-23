within FIRE_Modelica.Systems.Sensing.Barometer;

model Sensor "Ambient pressure/temperature and geometric altitude; no pressure-height inversion"
  parameter Real samplePeriod(unit="s") = 0.02;
  parameter Real pressureBias(unit="Pa") = 0;
  parameter Real temperatureBias(unit="K") = 0;
  parameter Real altitudeBias(unit="m") = 0;
  input Real ambientPressure(unit="Pa");
  input Real ambientTemperature(unit="K");
  input Real altitude_w(unit="m");
  input Real climbRate_w(unit="m/s");
  discrete output Real pressure(unit="Pa", start=101325, fixed=true);
  discrete output Real temperature(unit="K", start=288.15, fixed=true);
  discrete output Real altitude(unit="m", start=0, fixed=true);
  discrete output Real climbRate(unit="m/s", start=0, fixed=true);
  discrete output Real sampleTime(unit="s", start=0, fixed=true);

  replaceable model Measurement = BaseClasses.IdealMeasurement
    constrainedby BaseClasses.PartialMeasurement
    "Continuous measurement model in the family channel order"
    annotation(choicesAllMatching=true);
  replaceable model Response = FIRE_Modelica.Systems.Sensing.ResponseModels.Ideal
    constrainedby FIRE_Modelica.Systems.Sensing.ResponseModels.PartialResponse
    "Continuous response before bias and acquisition"
    annotation(choicesAllMatching=true);
  Measurement measurement;
  Response response(final n=4);
initial equation
  assert(samplePeriod > 0, "Barometer samplePeriod must be positive");
equation
  measurement.ambientPressure = ambientPressure;
  measurement.ambientTemperature = ambientTemperature;
  measurement.altitude_w = altitude_w;
  measurement.climbRate_w = climbRate_w;
  response.u = measurement.value;
  when sample(0, samplePeriod) then
    pressure = response.y[1] + pressureBias;
    temperature = response.y[2] + temperatureBias;
    altitude = response.y[3] + altitudeBias;
    climbRate = response.y[4];
    sampleTime = time;
  end when;
end Sensor;
