within FIRE_Modelica.Data.Presets;
record HexaX
  extends AirframeGeometry(nArms=6,nRotors=6,nActuators=6,
    armMount=Utilities.Math.regularPolygon(6,0.3,0,Modelica.Constants.pi/6));
end HexaX;
