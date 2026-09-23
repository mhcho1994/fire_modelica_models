within fire_modelica_models.Systems.Sensing.ResponseModels;

model Ideal "Immediate response without additional states"
  extends PartialResponse;
equation
  y = u;
end Ideal;
