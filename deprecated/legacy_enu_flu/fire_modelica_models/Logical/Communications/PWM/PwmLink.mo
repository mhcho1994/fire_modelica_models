within fire_modelica_models.Logical.Communications.PWM;

partial model PwmLink
  parameter PwmConfig config "PWM configuration";
  PwmBus bus annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
end PwmLink;
