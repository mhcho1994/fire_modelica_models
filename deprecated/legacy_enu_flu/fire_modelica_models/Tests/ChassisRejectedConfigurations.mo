within fire_modelica_models.Tests;

package ChassisRejectedConfigurations "Negative fixtures: each nested model must fail validation"
  model DuplicateIds "Expected error: Duplicate physical mass componentId"
    fire_modelica_models.Physical.Mechanical.Chassis.ChassisAssembly chassis(
      useAssembledMass = true,
      core(componentId = "same-physical-part"),
      nPayloads = 1,
      payloads = {fire_modelica_models.Physical.Mechanical.Chassis.Payloads.FixedPayload(
        mass = 0.3, componentId = "same-physical-part")});
    output Real totalMass = chassis.mass;
  end DuplicateIds;

  model ImproperRotation "Expected error: Part inertia axes must be a right-handed rotation"
    fire_modelica_models.Physical.Mechanical.Chassis.ChassisAssembly chassis(
      useAssembledMass = true, core(R_Cj = diagonal({1, 1, -1})));
    output Real totalMass = chassis.mass;
  end ImproperRotation;

  model IndefiniteInertia "Expected error: Part inertia must represent a physical mass distribution"
    fire_modelica_models.Physical.Mechanical.Chassis.ChassisAssembly chassis(
      useAssembledMass = true, core(inertia = [1, 2, 0; 2, 1, 0; 0, 0, 1]));
    output Real totalMass = chassis.mass;
  end IndefiniteInertia;

  model NonphysicalPrincipalMoments "Positive inertia alone is insufficient: 3 > 1 + 1"
    fire_modelica_models.Physical.Mechanical.Chassis.ChassisAssembly chassis(
      useAssembledMass = true, core(inertia = diagonal({1, 1, 3})));
    output Real totalMass = chassis.mass;
  end NonphysicalPrincipalMoments;

  model MasslessInertia "Expected error: A zero-mass part cannot carry nonzero mass inertia"
    fire_modelica_models.Physical.Mechanical.Chassis.ChassisAssembly chassis(
      useAssembledMass = true, nAdditionalParts = 1,
      additionalParts = {fire_modelica_models.Physical.Mechanical.MassProperties(mass = 0, inertia = identity(3))});
    output Real totalMass = chassis.mass;
  end MasslessInertia;
end ChassisRejectedConfigurations;
