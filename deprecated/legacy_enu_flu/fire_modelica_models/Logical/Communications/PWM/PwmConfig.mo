within fire_modelica_models.Logical.Communications.PWM;

record PwmConfig
  parameter Real frequency(min = 0) = 50 "PWM signal frequency [Hz]";
  parameter Real pulseMin(min = 0) = 1000e-6 "Minimum pulse width [s]";
  parameter Real pulseTrim(min = 0) = 1500e-6 "Trim pulse width [s]";
  parameter Real pulseMax(min = 0) = 2000e-6 "Maximum pulse width [s]";
  parameter ActiveLevel activeLevel = ActiveLevel.activeHigh "Active pulse level";
end PwmConfig;
