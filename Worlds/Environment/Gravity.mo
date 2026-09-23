within FIRE_Modelica.Worlds.Environment;

model Gravity
  parameter Real g = FIRE_Modelica.Utilities.Constants.g
    "Gravity magnitude [m/s2]";
  Modelica.Blocks.Interfaces.RealOutput gravity[3];
equation
  gravity = {0, 0, -g};
end Gravity;
