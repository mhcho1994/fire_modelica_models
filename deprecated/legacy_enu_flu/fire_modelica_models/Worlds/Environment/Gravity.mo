within fire_modelica_models.Worlds.Environment;

model Gravity
  parameter Real g = fire_modelica_models.Utilities.Constants.g
    "Gravity magnitude [m/s2]";
  Modelica.Blocks.Interfaces.RealOutput gravity[3];
equation
  gravity = {0, 0, -g};
end Gravity;
