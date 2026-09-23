within fire_modelica_models.Logical.Communications.UART;

partial model UartLink
  parameter UartConfig config "UART configuration";
  UartBus bus annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
end UartLink;
