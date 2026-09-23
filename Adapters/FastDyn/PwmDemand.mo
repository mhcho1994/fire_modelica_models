within FIRE_Modelica.Adapters.FastDyn;
model PwmDemand "N-channel pulse widths in microseconds to held [0,1] speed demand"
  parameter Integer nChannels(min=1)=4;
  parameter Real pwmMin[nChannels]=fill(1000,nChannels);
  parameter Real pwmMax[nChannels]=fill(2000,nChannels);
  parameter Real samplePeriod(min=Modelica.Constants.small)=0.0025;
  input Real pulseWidth_us[nChannels];
  output Real demand[nChannels](each start=0,each fixed=true);
  output Real sampleTime(start=0,fixed=true);
initial equation
  assert(samplePeriod>0,"Command sampling period must be positive");
  for i in 1:nChannels loop
    assert(pwmMax[i]>pwmMin[i],"PWM range must be positive");
  end for;
equation
  when sample(0,samplePeriod) then
    for i in 1:nChannels loop
      demand[i]=min(1,max(0,(pulseWidth_us[i]-pwmMin[i])/(pwmMax[i]-pwmMin[i])));
    end for;
    sampleTime=time;
  end when;
end PwmDemand;
