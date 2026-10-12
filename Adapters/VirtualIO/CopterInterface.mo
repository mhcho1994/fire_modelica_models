within fire_modelica_models.Adapters.VirtualIO;
partial model CopterInterface "Common P0/P1 VirtualIO PWM and sensor boundary; composer binds plant semantics"
  extends PartialCopterInterface;
  PwmInput commands(nChannels=geometry.nActuators,
    pwmMin=fill(pwm_min,geometry.nActuators),pwmMax=fill(pwm_max,geometry.nActuators));
equation
  commands.pulseWidth_us=pwm;
end CopterInterface;
