within fire_modelica_models.Systems.Sensing.GNSS;

model Sensor "Local Cartesian antenna position and velocity, sampled together"
  parameter Boolean sampled = true "False exposes the continuous sensor response";
  parameter Real samplePeriod(unit="s") = 0.2;
  parameter Real positionBias[3](each unit="m") = zeros(3);
  parameter Real velocityBias[3](each unit="m/s") = zeros(3);
  input Real position_w[3](each unit="m");
  input Real velocity_w[3](each unit="m/s");
  output Real position[3](each unit="m", each start=0, each fixed=sampled);
  output Real velocity[3](each unit="m/s", each start=0, each fixed=sampled);
  output Real sampleTime(unit="s", start=0, fixed=sampled);

  replaceable model Measurement = BaseClasses.IdealMeasurement
    constrainedby BaseClasses.PartialMeasurement
    "Continuous measurement model in the family channel order"
    annotation(choicesAllMatching=true);
  replaceable model Response = fire_modelica_models.Systems.Sensing.ResponseModels.Ideal
    constrainedby fire_modelica_models.Systems.Sensing.ResponseModels.PartialResponse
    "Continuous response before bias and acquisition"
    annotation(choicesAllMatching=true);
  Measurement measurement;
  Response response(final n=6);
initial equation
  assert(samplePeriod > 0, "GNSS samplePeriod must be positive");
equation
  measurement.position_w = position_w;
  measurement.velocity_w = velocity_w;
  response.u = measurement.value;
  if sampled then
    when sample(0, samplePeriod) then
      position = response.y[1:3] + positionBias;
      velocity = response.y[4:6] + velocityBias;
      sampleTime = time;
    end when;
  else
    position = response.y[1:3] + positionBias;
    velocity = response.y[4:6] + velocityBias;
    sampleTime = time;
  end if;
end Sensor;
