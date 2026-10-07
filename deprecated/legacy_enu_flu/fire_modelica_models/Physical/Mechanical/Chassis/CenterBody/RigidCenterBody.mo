within fire_modelica_models.Physical.Mechanical.Chassis.CenterBody;

record RigidCenterBody "Central frame mass; motion is owned by the vehicle rigid body"
  extends fire_modelica_models.Physical.Mechanical.MassProperties(
    mass = 1,
    inertia = diagonal({0.01, 0.01, 0.01}));
end RigidCenterBody;
