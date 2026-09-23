# fire-compose

Rust frontend for validated FIRE vehicle TOML. It emits a standard Modelica
assembly and provenance manifest; the compiler still owns Modelica semantics.

```sh
cargo run --locked --manifest-path tools/fire-compose/Cargo.toml -- validate configs/quad.toml --json
cargo run --locked --manifest-path tools/fire-compose/Cargo.toml -- emit configs/quad.toml
cargo run --locked --manifest-path tools/fire-compose/Cargo.toml -- check configs/quad.toml
```

When the physics configuration is embedded in a larger TOML file, pass
`--config-table FMU.models.quad.physics` to `validate`, `emit`, or `check`.
The path uses TOML dotted-key syntax (quoted key segments are supported).
Only the selected table is validated against the physics schema; other tables
may hold host/firmware settings. The manifest hashes the entire input TOML.
Standalone physics TOML files continue to work without this option.

`plant` (default) exposes normalized `demand[nActuators]`. `--profile fastdyn`
adds the flat PWM/sensor boundary used by FastDyn. `--output-dir` must be under
FIRE's `build/` directory or outside the source tree. Hand-authored and symlink
output files are protected. Only declared packages are staged under
`sources/fire_modelica_models/`, matching the canonical Modelica package name.
Existing outputs generated with the former `FIRE_Modelica` namespace can be
regenerated in place; an owned staged tree with the old name is removed after
the new tree is ready. Direct OpenModelica loading of the checkout requires its
directory to be named `fire_modelica_models`.

Schema 1 retains the Python generator's configuration model. Schema 2 also
accepts `acquisition = "sampled" | "continuous"` (default sampled), `[models]`,
and `[interface]`. Rotor selection currently supports `speed_driven`; sensor
responses support `ideal` or `first_order` with scalar/per-channel `tau`.
The FastDyn boundary can set `pwm_min`, `pwm_max`, `actuatorSamplePeriod`, `lat0`,
`lon0`, `ground_alt_wgs84`, and `earth_radius_m` via `[interface]`.

Continuous acquisition removes sensor/PWM sample-and-hold; it does not remove
response dynamics or landing-contact events. `sampleTime` becomes evaluation
time. Existing sensor/PWM classes default to their original sampled behavior.
`check` runs OpenModelica checkModel and translateModel; it is not an FMU test.

`composition.rs` is the domain-independent source representation; `config.rs`
and `lower.rs` retain FIRE knowledge. No Rumoca compiler crate is needed to
compose. The emitted `.mo` and manifest are build artifacts, not library sources.
