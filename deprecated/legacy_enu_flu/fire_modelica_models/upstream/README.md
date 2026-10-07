# Design provenance

This change implements conventional rigid-body, parallel-axis, first-order motor,
quadratic rotor, rigid sensor-mount, and unilateral compliant-contact equations
inside FIRE. No source file or implementation body from the repositories below
was copied or vendored in this update. Their full package trees are not runtime
dependencies. Existing FIRE source and the user's communications/servo work were
retained or migrated.

The architecture proposal reviewed these references:

| Reference | Inspected revision | Role |
|---|---|---|
| [CogniPilot/modelica_models](https://github.com/CogniPilot/modelica_models/tree/bc50718e9d89328e585230767e5a1547b8a1221e) | `bc50718e9d89328e585230767e5a1547b8a1221e` | Quaternion rigid-body conventions and physical validation cases |
| [ALSETLab/Modelica-Drone-3D-FMI](https://github.com/ALSETLab/Modelica-Drone-3D-FMI/tree/6fbfe933af0c878f9308c38eb93e588a1fb5cdd9) | `6fbfe933af0c878f9308c38eb93e588a1fb5cdd9` | Mechanical/electrical domain separation and assemblies |
| [RotorPy](https://github.com/spencerfolk/rotorpy/tree/40b09697a6a1596d404a67f459941d46a39eedd3) | `40b09697a6a1596d404a67f459941d46a39eedd3` | Future aerodynamic/flapping extension boundary; not implemented here |

If code is imported later, record its exact source revision and retain that
source's required license and notices with the imported implementation. These
architectural references do not assign a new license to the existing FIRE repo.
