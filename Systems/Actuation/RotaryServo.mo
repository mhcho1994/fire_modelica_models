within FIRE_Modelica.Systems.Actuation;

model RotaryServo
  "Rotary servo with angle limits and first-order lag"

  parameter Modelica.Units.SI.Angle angleMin =
    -30 * Modelica.Constants.pi / 180
    "Minimum servo angle";

  parameter Modelica.Units.SI.Angle angleMax =
     30 * Modelica.Constants.pi / 180
    "Maximum servo angle";

  parameter Modelica.Units.SI.Time timeConstant = 0.05
    "First-order time constant";

  Modelica.Blocks.Interfaces.RealInput angleCommand(
    unit = "rad")
    "Commanded servo angle";

  Modelica.Blocks.Interfaces.RealOutput angle(
    unit = "rad")
    "Actual servo angle and control-surface deflection";

protected
  Modelica.Units.SI.Angle limitedCommand;

initial equation
  angle = 0;

equation
  limitedCommand =
    min(max(angleCommand, angleMin), angleMax);

  timeConstant * der(angle) + angle =
    limitedCommand;

end RotaryServo;