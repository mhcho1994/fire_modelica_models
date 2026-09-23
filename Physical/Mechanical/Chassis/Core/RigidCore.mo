within FIRE_Modelica.Physical.Mechanical.Chassis.Core;

record RigidCore "Central frame mass; motion is owned by the vehicle rigid body"
  extends FIRE_Modelica.Data.MassProperties(
    mass = 1,
    inertia = diagonal({0.01, 0.01, 0.01}));
end RigidCore;
