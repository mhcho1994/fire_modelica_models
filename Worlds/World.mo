within FIRE_Modelica.Worlds;

model World
  Environment.Gravity gravityModel;
  Environment.MagneticField magneticFieldModel;
  Environment.Wind windModel;
  Environment.Atmosphere atmosphereModel;
  Interfaces.WorldBus world annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));
equation
  connect(world.gravity, gravityModel.gravity);
  connect(world.magneticField, magneticFieldModel.magneticField);
  connect(world.wind, windModel.wind);
  connect(world.temperature, atmosphereModel.temperature);
  connect(world.pressure, atmosphereModel.pressure);
  connect(world.density, atmosphereModel.density);
end World;
