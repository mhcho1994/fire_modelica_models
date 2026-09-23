within fire_modelica_models.Systems.Sensing.Magnetometer;

model Sensor "Sampled mounted magnetic field; magnetic inputs and bias are in Tesla"
  parameter Boolean sampled = true "False exposes the continuous sensor response";
  parameter Real samplePeriod(unit="s") = 0.02;
  parameter Real R_bs[3,3] = identity(3) "Sensor to body rotation";
  parameter Real bias[3](each unit="T") = zeros(3);
  input Real R_wb[3,3];
  input Real magneticField_w[3](each unit="T");
  output Real magneticField[3](each unit="T", each start=0, each fixed=sampled);
  output Real sampleTime(unit="s", start=0, fixed=sampled);

  replaceable model Measurement = BaseClasses.IdealMeasurement
    constrainedby BaseClasses.PartialMeasurement
    "Continuous measurement model in the family channel order"
    annotation(choicesAllMatching=true);
  replaceable model Response = fire_modelica_models.Systems.Sensing.ResponseModels.Ideal
    constrainedby fire_modelica_models.Systems.Sensing.ResponseModels.PartialResponse
    "Continuous response before bias and acquisition"
    annotation(choicesAllMatching=true);
  Measurement measurement(final R_bs=R_bs);
  Response response(final n=3);
initial equation
  assert(samplePeriod > 0, "Magnetometer samplePeriod must be positive");
equation
  measurement.R_wb = R_wb;
  measurement.magneticField_w = magneticField_w;
  response.u = measurement.value;
  if sampled then
    when sample(0, samplePeriod) then
      magneticField = response.y + bias;
      sampleTime = time;
    end when;
  else
    magneticField = response.y + bias;
    sampleTime = time;
  end if;
end Sensor;
