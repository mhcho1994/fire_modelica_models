within FIRE_Modelica.Physical.Mechanical.Aerodynamics;
model BodyDrag "Dissipative diagonal body drag applied at the CG"
  parameter Real linearDrag[3](each unit = "N.s/m") = zeros(3);
  parameter Modelica.Units.SI.Area dragArea[3] = zeros(3);
  input Modelica.Units.SI.Density rho "Local air density";
  input Modelica.Units.SI.Velocity airVelocity_b[3]
    "Body CG velocity relative to air, resolved in body";
  output Modelica.Units.SI.Force force_b[3];
initial equation
  assert(min(linearDrag) >= 0 and min(dragArea) >= 0,
    "Drag coefficients must be nonnegative");
equation
  assert(rho >= 0, "Air density must be nonnegative");
  // The exact zero-speed branch avoids 0/0 in generated norm derivatives.
  // This arithmetic guard does not suppress any ground-contact event.
  force_b = -linearDrag .* airVelocity_b
    - 0.5 * rho * (if noEvent(airVelocity_b * airVelocity_b > 0) then
      sqrt(airVelocity_b * airVelocity_b) else 0) * (dragArea .* airVelocity_b);
end BodyDrag;
