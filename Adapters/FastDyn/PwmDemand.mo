within fire_modelica_models.Adapters.FastDyn;
model PwmDemand "Continuous pulse-width normalization to target-speed ratio (not thrust fraction)"
  parameter Integer nChannels(min=1)=4;
  parameter Real pwmMin[nChannels]=fill(1000,nChannels) "microseconds";
  parameter Real pwmMax[nChannels]=fill(2000,nChannels) "microseconds";
  input Real pulseWidth_us[nChannels] "microseconds";
  output Real demand[nChannels](each unit="1") "Normalized target shaft speed";
initial equation
  for i in 1:nChannels loop
    assert(pwmMax[i]>pwmMin[i],"PWM range must be positive");
  end for;
equation
  for i in 1:nChannels loop
    demand[i]=noEvent(min(1,max(0,(pulseWidth_us[i]-pwmMin[i])/(pwmMax[i]-pwmMin[i]))));
  end for;
end PwmDemand;
