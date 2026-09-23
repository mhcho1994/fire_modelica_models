within FIRE_Modelica.Vehicles.Copter.Presets;
record OctoX
  extends Geometry(nArms=8,nRotors=8,nActuators=8,
    armMount=Utilities.Math.regularPolygon(8,0.35,0,Modelica.Constants.pi/8));
end OctoX;
