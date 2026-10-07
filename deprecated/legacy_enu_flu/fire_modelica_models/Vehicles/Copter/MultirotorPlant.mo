within fire_modelica_models.Vehicles.Copter;
model MultirotorPlant "N-rotor rigid vehicle, ENU world / FLU at combined CG"
  parameter Vehicles.Copter.Geometry geometry;
  parameter Boolean useAssembledMass = false;
  parameter Physical.Mechanical.MassProperties aggregate(mass=1.5,inertia=diagonal({0.02,0.02,0.04}));
  parameter Physical.Mechanical.Chassis.CenterBody.RigidCenterBody core(mass=1.2,inertia=diagonal({0.015,0.015,0.025}));
  parameter Physical.Mechanical.Chassis.Arms.RigidArm arms[geometry.nArms](
    each mass=0.05,r_C=geometry.armMount/2,each inertia=diagonal({0.0001,0.0001,0.0002}));
  parameter Integer nPayloads(min=0) = 0;
  parameter Physical.Mechanical.Chassis.Payloads.FixedPayload payloads[nPayloads];
  parameter Integer nAdditionalParts(min=0) = 0;
  parameter Physical.Mechanical.MassProperties additionalParts[nAdditionalParts];
  parameter Real omegaMax[geometry.nRotors] = fill(1000,geometry.nRotors) "rad/s";
  parameter Real motorTau[geometry.nRotors] = fill(0.03,geometry.nRotors) "s";
  parameter Real kT[geometry.nRotors] = fill(1e-5,geometry.nRotors) "N/(rad/s)^2";
  parameter Real kQ[geometry.nRotors] = fill(1.5e-7,geometry.nRotors) "N.m/(rad/s)^2";
  parameter Real rotorSpeed_start[geometry.nRotors] = zeros(geometry.nRotors);
  parameter Real p_start[3] = {0,0,1};
  parameter Real v_start[3] = zeros(3) "Body velocity at CG";
  parameter Real q_start[4] = {1,0,0,0};
  parameter Real omega_start[3] = zeros(3);
  parameter Real legStiffness = 1500 "N/m per leg";
  parameter Real legDamping = 25 "N.s/m per leg";
  parameter Real tangentialDamping = 10 "N.s/m per leg";
  parameter Real frictionCoefficient = 0.6;
  parameter Real linearDrag[3] = zeros(3) "N.s/m";
  parameter Real dragArea[3] = zeros(3) "Cd*A, m2";
  input Real demand[geometry.nActuators] "Normalized rotor speed command [0,1], mapped by actuatorIndex";
  input Real gravity_w[3] = {0,0,-9.80665};
  input Real wind_w[3] = zeros(3);
  input Real density = 1.225;
  input Real externalForce_b[3] = zeros(3) "Non-gravity force at CG";
  input Real externalMoment_b[3] = zeros(3) "Moment about combined CG";
  input Real terrainPoint_w[3] = zeros(3);
  input Real terrainNormal_w[3] = {0,0,1};
  input Real terrainVelocity_w[3] = zeros(3);
  replaceable model RotorUnit = Systems.Propulsion.SpeedDrivenRotor
    constrainedby Systems.Propulsion.PartialSpeedDrivenRotor;
  Physical.Mechanical.Chassis.ChassisAssembly chassis(
    useAssembledMass=useAssembledMass,aggregate=aggregate,core=core,
    nArms=geometry.nArms,arms=arms,nPayloads=nPayloads,payloads=payloads,
    nAdditionalParts=nAdditionalParts,additionalParts=additionalParts);
  final parameter Real rotorPosition_b[geometry.nRotors,3] =
    {geometry.rotorPosition_C[i,:]-chassis.cg_C for i in 1:geometry.nRotors};
  final parameter Real legPosition_b[geometry.nLegs,3] =
    {geometry.legPosition_C[i,:]-chassis.cg_C for i in 1:geometry.nLegs};
  Physical.Mechanical.Dynamics.RigidBody6DOF body(mass=chassis.mass,inertia=chassis.inertia,
    p_start=p_start,v_start=v_start,q_start=q_start,omega_start=omega_start);
  RotorUnit rotors[geometry.nRotors](omegaMax=omegaMax,spinSign=geometry.spinSign,
    tau=motorTau,kT=kT,kQ=kQ,omega_start=rotorSpeed_start);
  Physical.Mechanical.Chassis.LandingGear.LandingGearAssembly landingGear(
    nLegs=geometry.nLegs,rLeg_b=legPosition_b,stiffness=legStiffness,damping=legDamping,
    tangentialDamping=tangentialDamping,frictionCoefficient=frictionCoefficient);
  Physical.Mechanical.Aerodynamics.BodyDrag drag(linearDrag=linearDrag,dragArea=dragArea);
  output Interfaces.VehicleTruth truth;
  output Real force_b[3] "Sum of non-gravity loads";
  output Real moment_b[3] "Sum about combined CG";
  output Real rotorForce_b[geometry.nRotors,3];
  output Real rotorMoment_b[geometry.nRotors,3];
initial equation
  for i in 1:geometry.nRotors loop
    assert(geometry.rotorArmIndex[i]>=1 and geometry.rotorArmIndex[i]<=geometry.nArms,"Invalid rotor arm index");
    assert(geometry.actuatorIndex[i]>=1 and geometry.actuatorIndex[i]<=geometry.nActuators,"Invalid rotor actuator index");
    assert(max(abs(transpose(geometry.R_br[i,:,:])*geometry.R_br[i,:,:]-identity(3)))<1e-9
      and abs(Modelica.Math.Matrices.det(geometry.R_br[i,:,:])-1)<1e-9,"Rotor mount must be a proper rotation");
  end for;
equation
  for i in 1:geometry.nRotors loop
    rotors[i].demand = demand[geometry.actuatorIndex[i]];
    rotors[i].airVelocity_r = transpose(geometry.R_br[i,:,:])*(body.v_b+
      cross(body.omega_b,rotorPosition_b[i,:])-transpose(body.R_wb)*wind_w);
    rotors[i].omegaBody_r = transpose(geometry.R_br[i,:,:])*body.omega_b;
    rotorForce_b[i,:] = geometry.R_br[i,:,:]*rotors[i].force_r;
    rotorMoment_b[i,:] = cross(rotorPosition_b[i,:],rotorForce_b[i,:])+
      geometry.R_br[i,:,:]*rotors[i].moment_r;
  end for;
  landingGear.p_w = body.p_w;
  landingGear.v_b = body.v_b;
  landingGear.omega_b = body.omega_b;
  landingGear.R_wb = body.R_wb;
  landingGear.terrainPoint_w = terrainPoint_w;
  landingGear.terrainNormal_w = terrainNormal_w;
  landingGear.terrainVelocity_w = terrainVelocity_w;
  drag.airVelocity_b = body.v_b-transpose(body.R_wb)*wind_w;
  drag.rho = density;
  for j in 1:3 loop
    force_b[j] = sum(rotorForce_b[:,j])+landingGear.force_b[j]+drag.force_b[j]+externalForce_b[j];
    moment_b[j] = sum(rotorMoment_b[:,j])+landingGear.moment_b[j]+externalMoment_b[j];
  end for;
  body.force_b = force_b;
  body.moment_b = moment_b;
  body.gravity_w = gravity_w;
  truth.p_w = body.p_w;
  truth.v_w = body.v_w;
  truth.v_b = body.v_b;
  truth.q_wb = body.q_wb;
  truth.R_wb = body.R_wb;
  truth.omega_b = body.omega_b;
  truth.a_w = body.a_w;
  truth.alpha_b = body.alpha_b;
end MultirotorPlant;
