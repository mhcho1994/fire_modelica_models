within FIRE_Modelica.Tests;

package ChassisRejectedConfigurations "Negative fixtures: each nested model must fail validation"
  model DuplicateIds "Expected error: Duplicate physical mass componentId"
    FIRE_Modelica.Physical.Mechanical.Chassis.ChassisAssembly chassis(
      useAssembledMass = true,
      core(componentId = "same-physical-part"),
      nPayloads = 1,
      payloads = {FIRE_Modelica.Physical.Mechanical.Chassis.Payloads.FixedPayload(
        mass = 0.3, componentId = "same-physical-part")});
    output Real totalMass = chassis.mass;
  end DuplicateIds;

  model ImproperRotation "Expected error: Part inertia axes must be a right-handed rotation"
    FIRE_Modelica.Physical.Mechanical.Chassis.ChassisAssembly chassis(
      useAssembledMass = true, core(R_Cj = diagonal({1, 1, -1})));
    output Real totalMass = chassis.mass;
  end ImproperRotation;

  model IndefiniteInertia "Expected error: Part inertia must represent a physical mass distribution"
    FIRE_Modelica.Physical.Mechanical.Chassis.ChassisAssembly chassis(
      useAssembledMass = true, core(inertia = [1, 2, 0; 2, 1, 0; 0, 0, 1]));
    output Real totalMass = chassis.mass;
  end IndefiniteInertia;

  model NonphysicalPrincipalMoments "Positive inertia alone is insufficient: 3 > 1 + 1"
    FIRE_Modelica.Physical.Mechanical.Chassis.ChassisAssembly chassis(
      useAssembledMass = true, core(inertia = diagonal({1, 1, 3})));
    output Real totalMass = chassis.mass;
  end NonphysicalPrincipalMoments;

  model MasslessInertia "Expected error: A zero-mass part cannot carry nonzero mass inertia"
    FIRE_Modelica.Physical.Mechanical.Chassis.ChassisAssembly chassis(
      useAssembledMass = true, nAdditionalParts = 1,
      additionalParts = {FIRE_Modelica.Physical.Mechanical.MassProperties(mass = 0, inertia = identity(3))});
    output Real totalMass = chassis.mass;
  end MasslessInertia;
end ChassisRejectedConfigurations;
