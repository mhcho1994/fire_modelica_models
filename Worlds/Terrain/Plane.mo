within fire_modelica_models.Worlds.Terrain;

model Plane "Horizontal or inclined translating plane with a unit outward normal"
  parameter Real point_w[3](each unit = "m") = {0, 0, 0}
    "Point on the plane at time zero";
  parameter Real normal_w[3] = {0, 0, 1};
  parameter Real velocity_w[3](each unit = "m/s") = {0, 0, 0};
  output Real surfacePoint_w[3](each unit = "m");
  output Real surfaceNormal_w[3];
  output Real surfaceVelocity_w[3](each unit = "m/s");
initial equation
  assert(abs(normal_w * normal_w - 1) < 1e-8, "Plane normal must be a unit vector");
equation
  surfacePoint_w = point_w + time * velocity_w;
  surfaceNormal_w = normal_w;
  surfaceVelocity_w = velocity_w;
end Plane;
