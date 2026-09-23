within fire_modelica_models.Logical.Communications.UART;

record UartConfig
  parameter Integer baudRate(min = 1) = 115200 "UART baud rate [bit/s]";
  parameter Integer dataBits(min = 5, max = 9) = 8 "Number of data bits";
  parameter Parity parity = Parity.none "Parity mode";
  parameter Integer stopBits(min = 1, max = 2) = 1 "Number of stop bits";
end UartConfig;
