within FIRE_Modelica.Tests;

model ExportOcto "FMU boundary fixture with eight independent PWM inputs and observable contact"
  extends FIRE_Modelica.Adapters.FastDyn.MultirotorFmu(
    geometry=FIRE_Modelica.Data.Presets.OctoX(), p_start={0,0,0.5});
  output Boolean contact[geometry.nLegs];
  output Real gap[geometry.nLegs];
  output Real normalForce[geometry.nLegs];
  output Real sampledDemand[geometry.nActuators];
  output Real commandSampleTime;
equation
  contact = adapter.vehicle.landingGear.contact;
  gap = adapter.vehicle.landingGear.gap;
  normalForce = adapter.vehicle.landingGear.normalForce;
  sampledDemand = adapter.commands.demand;
  commandSampleTime = adapter.commands.sampleTime;
end ExportOcto;
