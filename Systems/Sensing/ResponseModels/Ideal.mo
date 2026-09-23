within FIRE_Modelica.Systems.Sensing.ResponseModels;

model Ideal "Immediate response without additional states"
  extends PartialResponse;
equation
  y = u;
end Ideal;
