within fire_modelica_models.Logical.Communications.SPI;

record SpiConfig
  parameter Integer clockRate(min = 1) = 1000000 "SPI clock rate [Hz]";
  parameter ClockPolarity polarity = ClockPolarity.idleLow "Clock polarity";
  parameter ClockPhase phase = ClockPhase.firstEdge "Clock phase";
  parameter Integer wordLength(min = 1) = 8 "Bits per transfer word";
end SpiConfig;
