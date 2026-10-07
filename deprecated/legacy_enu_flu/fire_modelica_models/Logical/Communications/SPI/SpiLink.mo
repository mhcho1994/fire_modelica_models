within fire_modelica_models.Logical.Communications.SPI;

partial model SpiLink
  parameter SpiConfig config "SPI configuration";
  SpiBus bus annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
end SpiLink;
