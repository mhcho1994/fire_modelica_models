within fire_modelica_models.Vehicles.Copter.Presets;
record HexaX
  extends Geometry(nArms=6,nRotors=6,nActuators=6,
    armMount=Utilities.Math.regularPolygon(6,0.3,0,Modelica.Constants.pi/6));
end HexaX;
