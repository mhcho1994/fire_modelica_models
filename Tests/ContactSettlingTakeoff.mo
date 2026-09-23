within FIRE_Modelica.Tests;

model ContactSettlingTakeoff "Drop onto a compliant leg, settle, and lift off without state resets"
  parameter Real mass = 2;
  parameter Real g = 9.81;
  parameter Real stiffness = 1000;
  FIRE_Modelica.Physical.Mechanical.Chassis.LandingGear.LandingGearAssembly gear(
    nLegs = 1, rLeg_b = {{0, 0, 0}}, stiffness = stiffness, damping = 60);
  Real z(start = 0.2, fixed = true);
  Real v(start = 0, fixed = true);
  Real appliedThrust;
  Real contactDissipation;
  output Real energy;
equation
  gear.p_w = {0, 0, z};
  gear.v_b = {0, 0, v};
  gear.omega_b = zeros(3);
  gear.R_wb = identity(3);
  gear.terrainPoint_w = zeros(3);
  gear.terrainNormal_w = {0, 0, 1};
  gear.terrainVelocity_w = zeros(3);
  appliedThrust = if time < 3 then 0 else 1.5 * mass * g;
  der(z) = v;
  mass * der(v) = gear.force_b[3] - mass * g + appliedThrust;
  energy = 0.5 * mass * v * v + mass * g * z + 0.5 * stiffness * max(0, -z)^2;
  contactDissipation = if gear.contact[1] then
    (gear.force_b[3] - stiffness * max(0, -z)) * v else 0;
  assert(gear.normalForce[1] >= 0, "Ground must never pull the vehicle down");
  assert(contactDissipation <= 1e-7, "Normal spring/damper contact must not generate energy");
  when time >= 2.5 then
    assert(gear.contact[1] and abs(v) < 1e-5 and abs(z + mass * g / stiffness) < 1e-5,
      "A dropped sprung mass must settle at the static spring compression");
  end when;
  when time >= 3.5 then
    assert(not gear.contact[1] and z > 0 and abs(gear.normalForce[1]) < 1e-12,
      "Sufficient upward thrust must release contact without a velocity reset");
  end when;
  annotation(experiment(StopTime = 4, Tolerance = 1e-8, Interval = 0.002));
end ContactSettlingTakeoff;
