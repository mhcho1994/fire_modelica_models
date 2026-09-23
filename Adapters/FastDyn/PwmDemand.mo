within fire_modelica_models.Adapters.FastDyn;
model PwmDemand "N-channel pulse widths in microseconds to held [0,1] speed demand"
  parameter Boolean sampled = true "False continuously converts the host-held PWM input";
  parameter Integer nChannels(min=1)=4;
  parameter Real pwmMin[nChannels]=fill(1000,nChannels);
  parameter Real pwmMax[nChannels]=fill(2000,nChannels);
  parameter Real samplePeriod(min=Modelica.Constants.small)=0.0025;
  input Real pulseWidth_us[nChannels];
  output Real demand[nChannels](each start=0,each fixed=sampled);
  output Real sampleTime(start=0,fixed=sampled);
initial equation
  assert(samplePeriod>0,"Command sampling period must be positive");
  for i in 1:nChannels loop
    assert(pwmMax[i]>pwmMin[i],"PWM range must be positive");
  end for;
equation
  if sampled then
    when sample(0,samplePeriod) then
      for i in 1:nChannels loop
        demand[i]=min(1,max(0,(pulseWidth_us[i]-pwmMin[i])/(pwmMax[i]-pwmMin[i])));
      end for;
      sampleTime=time;
    end when;
  else
    for i in 1:nChannels loop
      demand[i]=noEvent(min(1,max(0,(pulseWidth_us[i]-pwmMin[i])/(pwmMax[i]-pwmMin[i]))));
    end for;
    sampleTime=time;
  end if;
end PwmDemand;
