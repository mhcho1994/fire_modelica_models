within FIRE_Modelica.Systems.Sensing.ResponseModels;

model FirstOrder "Independent first-order channels; tau=0 bypasses a channel"
  extends PartialResponse;
  parameter Real tau[n](each unit="s", each min=0) = fill(0.01,n)
    "Compile-time channel time constants; this is not a transport delay";
initial equation
  for i in 1:n loop
    assert(tau[i] >= 0, "Sensor response tau must be nonnegative");
    if tau[i] > 0 then
      y[i] = u[i];
    end if;
  end for;
equation
  for i in 1:n loop
    if tau[i] > 0 then
      tau[i]*der(y[i]) = u[i] - y[i];
    else
      y[i] = u[i];
    end if;
  end for;
end FirstOrder;
