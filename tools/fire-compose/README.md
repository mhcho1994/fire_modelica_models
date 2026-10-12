# fire-compose

Rust configuration frontend for the minimum continuous NED/FRD multicopter core.
It emits standard Modelica source and a provenance manifest; the compiler owns
Modelica semantics. No Rumoca library is needed to compose.

```sh
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- validate configs/quad.toml --json
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- emit configs/hex.toml --profile vio
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- check configs/quad.toml
```

Only `schema_version=3` is accepted. World is NED; body/chassis is FRD. Presets preserve
previous physical rotor/mount order and East-facing initial heading. Explicit numeric
fields must already use the new frames. Schema 1/2 must be migrated with the transformations
in `docs/architecture.md`; changing only the schema number is insufficient.

`acquisition="continuous"` is optional and is the only supported value.
`[models]` may select `rotor="speed_driven"` (default P0), `rotor="voltage_driven"` (P1),
and `response="ideal"` for each sensor.
Other response profiles, legacy `[ground]` / `geometry.nLegs` settings, sample periods
and acquisition timestamps are deprecated. Static aggregate or assembled mass budgets remain supported.
Mass `componentId` values (and arm `componentIds`) remain configuration metadata.
The composer rejects duplicate nonempty IDs across the selected assembled parts before
emission and preserves the resolved IDs in the manifest's `config.mass`. Empty IDs are
allowed. Generated Modelica mass records contain only `mass`, `r_C`, `R_Cj` and `inertia`;
ID changes alone do not change the numerical model. Direct Modelica constructors no
longer accept `componentId`; callers bypassing the composer must ensure unique physical
parts themselves. Numerical mass/inertia validation remains in Modelica.

`[landing_gear]` accepts `enabled` (default true), `position_C` (exactly 4x3, FRD
relative to chassis C), `groundZ` (NED), `stiffness`, `damping`, `tangentialDamping`,
and `frictionCoefficient`. Coefficients are finite and nonnegative. Positions are
rebased by the combined CG in Modelica. The mass budget must already include physical
gear mass; no extra mass is added by the contact law. Set `enabled=false` for no contact.
Standalone initial position still defaults to NED z=-1 m. Bundled quad/hexa configs
explicitly start at z=-0.12 m, leaving default feet 2 cm above the plane.
The manifest records the chosen contact profile and required ground-contact events.

`[interface]` accepts pwm_min, pwm_max, lat0, lon0, ground_alt_wgs84, earth_radius_m.
Atmospheric pressure and temperature are constant in this minimum profile.

`plant` exposes P0 target-speed `demand[N]` or P1 modulation `escCommand[N]` and
`dcVoltage` (an input with a configured default, not an internal source at this boundary).
`--profile vio` adds the flat PWM/sensor boundary and binds P1 supply internally.
Both P0 and P1 extend `Adapters.VirtualIO.CopterInterface`; its shared `PwmInput` returns
`normalizedCommand`. The composer explicitly binds that to `plant.demand` or
`plant.escCommand`. `[interface] pwm_min/pwm_max` defaults to 1100/1900 us for both.
Legacy `--profile fastdyn` is accepted but emits canonical `profile="vio"` and identical
source. Manifest `input_adapter` records calibration, continuous normalization and
`plant_mapping`; `plant` profile has no PWM adapter metadata. Only the CLI alias is retained:
Modelica packages `Adapters.FastDyn` and `Adapters.VIO` are removed. Use `Adapters.VirtualIO`.
Regeneration replaces the staged library so obsolete package directories are removed.
For embedded physics, use `--config-table FMU.models.quad.physics` (TOML dotted-key syntax).
Other host tables are untouched. Their physics schema 1/2 is also rejected.
`--output-dir` must be under FIRE's build/ or outside the source tree. Hand-authored and
symlink outputs are protected. Only package.order-declared source is staged;
deprecated/ is excluded from source hashing and staging. Manifest version 2 records
`ned_frd_continuous_v1`, frame/profile metadata and separate validation statuses.
This is source composition, not arbitrary contract-aware RBC linking.

`check` runs OpenModelica checkModel and translateModel. Use `tools/probe_export.py`
for actual FMI 2.0 CS generation/stepping. Neither proves Rumoca/FastDyn firmware integration.

## P1 voltage-driven selection

```toml
schema_version = 3
model_name = "ConfiguredP1"
[models]
rotor = "voltage_driven"
[power]
dcVoltage = 16.0
[motor]
Kv = 62.5       # (rad/s)/V, required; NOT rpm/V
tauOmega = 0.03 # s
```

`power.dcVoltage` is a required finite nonnegative scalar for P1. `motor.Kv` is
required and positive; `tauOmega` is positive. `Kv`, `tauOmega`, `kT`, `kQ` and
`omega_start` may be scalar or length nRotors; initial speed is nonnegative.
P1 has no `omegaMax` speed cap. P0 rejects `[power]` and P1 motor fields; P1 rejects
P0 motor fields. This is an additive schema-3 extension, with existing P0 defaults retained.

The manifest records `physics.propulsion`, command contract, unipolar voltage-modulation
mode, input-default/internal-constant supply policy, SI speed gain, one speed state per
rotor, and absence of current/power/load feedback. These contracts document the bounded
composer's selection; they do not claim arbitrary RBC semantic matching.
Use `configs/quad_p1.toml` / `hex_p1.toml` for complete examples.
