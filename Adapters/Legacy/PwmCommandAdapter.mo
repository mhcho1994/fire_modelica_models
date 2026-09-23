within FIRE_Modelica.Adapters.Legacy;

model PwmCommandAdapter "Sample and normalize numeric PWM commands; no actuator dynamics"
  import FIRE_Modelica.Utilities.Math.clip;

  parameter Integer nChannels(min = 1) = 4
    "Number of PWM actuator channels";
  parameter Real pwmMin[nChannels] = fill(1000, nChannels)
    "Minimum PWM value for each channel";
  parameter Real pwmTrim[nChannels] = fill(1500, nChannels)
    "Trim PWM value for each channel";
  parameter Real pwmMax[nChannels] = fill(2000, nChannels)
    "Maximum PWM value for each channel";
  parameter Real samplePeriod = 0.02 "Actuator sample period [s]";

  Modelica.Blocks.Interfaces.RealInput pwmCommand[nChannels] annotation(
    Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Modelica.Blocks.Interfaces.RealOutput normalizedCommand[nChannels] annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));

protected
  discrete Real normalizedBuffer[nChannels](start = fill(0, nChannels));

equation
  normalizedCommand = pre(normalizedBuffer);

algorithm
  when sample(0, samplePeriod) then
    for i in 1:nChannels loop
      normalizedBuffer[i] := clip(
        (pwmCommand[i] - pwmTrim[i]) / max(pwmMax[i] - pwmTrim[i], FIRE_Modelica.Utilities.Constants.eps),
        -1,
        1);
    end for;
  end when;
end PwmCommandAdapter;
