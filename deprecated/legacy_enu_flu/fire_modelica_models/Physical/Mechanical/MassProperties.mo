within fire_modelica_models.Physical.Mechanical;

record MassProperties "A physical part's mass and inertia about its own centre of mass"
  Real mass(min = 0, unit = "kg") = 0;
  Real r_C[3](each unit = "m") = {0, 0, 0}
    "Part centre of mass in the fixed chassis reference frame C";
  Real R_Cj[3, 3] = identity(3)
    "Rotation from the part's inertia axes j to chassis axes C";
  Real inertia[3, 3](each unit = "kg.m2") = zeros(3, 3)
    "Inertia about the part centre of mass, resolved in axes j";
  String componentId = ""
    "Optional physical-part identity; nonempty IDs must be unique within a mass budget";
  annotation(Documentation(info = "<html><p>Use a stable, unique componentId when the same physical part is referenced by several subsystems. The chassis rejects duplicate nonempty IDs. A blank ID preserves compatibility with simple parameter constructors, but the caller remains responsible for including that physical mass only once.</p></html>"));
end MassProperties;
