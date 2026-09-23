within FIRE_Modelica.Logical.Communications.SPI;

expandable connector SpiBus
  Boolean sclk "Serial clock";
  Boolean mosi "Master out, slave in";
  Boolean miso "Master in, slave out";
  Boolean cs "Chip select";
end SpiBus;
