//! Domain validation and default resolution; validation has no filesystem effects.
use crate::{identifier, Result};
use serde::Serialize;
use std::collections::BTreeSet;
use toml::{map::Map, Value};

type Table = Map<String, Value>;
const IDENTITY: [[f64; 3]; 3] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];

#[derive(Clone, Debug, Serialize)]
pub struct ResolvedFireConfig {
    pub schema_version: i64,
    pub acquisition: String,
    pub model_name: String,
    pub preset: String,
    pub geometry: Table,
    pub mass: Table,
    pub motor: Table,
    pub sensors: Table,
    pub initial: Table,
    pub ground: Table,
    #[serde(skip_serializing_if = "Table::is_empty")]
    pub models: Table,
    #[serde(skip_serializing_if = "Table::is_empty")]
    pub interface: Table,
}

fn table<'a>(v: &'a Value, allowed: &[&str], label: &str) -> Result<&'a Table> {
    let t = v
        .as_table()
        .ok_or_else(|| format!("{label}: expected a table"))?;
    for k in t.keys() {
        if !allowed.contains(&k.as_str()) {
            return Err(format!("{label}: unknown fields {k}"));
        }
    }
    Ok(t)
}
fn section(t: &Table, key: &str) -> Value {
    t.get(key)
        .cloned()
        .unwrap_or_else(|| Value::Table(Table::new()))
}
fn number(v: &Value, label: &str, positive: bool, nonnegative: bool) -> Result<f64> {
    let x = v
        .as_float()
        .or_else(|| v.as_integer().map(|n| n as f64))
        .ok_or_else(|| format!("{label}: expected a finite number"))?;
    if !x.is_finite() || (positive && x <= 0.) || (nonnegative && x < 0.) {
        return Err(format!(
            "{label}: expected a finite {}number",
            if positive {
                "positive "
            } else if nonnegative {
                "nonnegative "
            } else {
                ""
            }
        ));
    }
    Ok(x)
}
fn integer(v: &Value, label: &str, min: i64) -> Result<usize> {
    let n = v
        .as_integer()
        .ok_or_else(|| format!("{label}: expected integer >= {min}"))?;
    if n < min {
        return Err(format!("{label}: expected integer >= {min}"));
    }
    usize::try_from(n).map_err(|_| format!("{label}: integer is too large"))
}
fn array(v: &Value, dims: &[usize], label: &str) -> Result<()> {
    if dims.is_empty() {
        number(v, label, false, false)?;
        return Ok(());
    }
    let a = v
        .as_array()
        .filter(|a| a.len() == dims[0])
        .ok_or_else(|| format!("{label}: expected dimensions {dims:?}"))?;
    for x in a {
        array(x, &dims[1..], label)?;
    }
    Ok(())
}
fn matrix(v: &Value, label: &str) -> Result<[[f64; 3]; 3]> {
    array(v, &[3, 3], label)?;
    let mut a = [[0.; 3]; 3];
    for (i, row) in a.iter_mut().enumerate() {
        for (j, x) in row.iter_mut().enumerate() {
            *x = number(&v[i][j], label, false, false)?;
        }
    }
    Ok(a)
}
fn det(a: &[[f64; 3]; 3]) -> f64 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}
fn rotation(v: &Value, label: &str) -> Result<()> {
    let a = matrix(v, label)?;
    let mut error: f64 = 0.;
    for i in 0..3 {
        for j in 0..3 {
            error = error.max(
                ((0..3).map(|k| a[k][i] * a[k][j]).sum::<f64>() - if i == j { 1. } else { 0. })
                    .abs(),
            );
        }
    }
    if error > 1e-9 || (det(&a) - 1.).abs() > 1e-9 {
        return Err(format!(
            "{label}: expected an orthonormal rotation with determinant +1"
        ));
    }
    Ok(())
}
fn inertia(v: &Value, label: &str) -> Result<()> {
    let a = matrix(v, label)?;
    let scale = a.iter().flatten().fold(0f64, |s, x| s.max(x.abs()));
    for (i, row) in a.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            if (value - a[j][i]).abs() > 1e-12 * scale.max(1e-30) {
                return Err(format!("{label}: inertia must be symmetric"));
            }
        }
    }
    let b = a.map(|r| r.map(|x| x / scale));
    if scale == 0. || b[0][0] <= 0. || b[0][0] * b[1][1] - b[0][1] * b[1][0] <= 0. || det(&b) <= 0.
    {
        return Err(format!("{label}: inertia must be positive definite"));
    }
    let half_trace = (b[0][0] + b[1][1] + b[2][2]) / 2.;
    let mut second = [[0.; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            second[i][j] = (if i == j { half_trace } else { 0. }) - b[i][j];
        }
    }
    let mut minors = vec![det(&second)];
    for i in 0..3 {
        minors.push(second[i][i]);
        for j in i + 1..3 {
            minors.push(second[i][i] * second[j][j] - second[i][j] * second[j][i]);
        }
    }
    if minors.iter().any(|x| *x < -1e-10) {
        return Err(format!(
            "{label}: inertia must represent a physical mass distribution"
        ));
    }
    Ok(())
}
fn component_id(v: &Value) -> Result<()> {
    let s = v.as_str().ok_or("componentId: expected string")?;
    if s.len() > 128
        || !s
            .bytes()
            .all(|c| c.is_ascii_alphanumeric() || b"_.:/-".contains(&c))
    {
        return Err("componentId: invalid characters or length".into());
    }
    Ok(())
}
fn vec3(v: [f64; 3]) -> Value {
    Value::Array(v.into_iter().map(Value::Float).collect())
}
fn mat(a: [[f64; 3]; 3]) -> Value {
    Value::Array(a.into_iter().map(vec3).collect())
}
fn int(n: usize) -> Value {
    Value::Integer(n as i64)
}
fn part(v: &Value, label: &str, position: [f64; 3]) -> Result<Value> {
    let mut p = table(v, &["mass", "inertia", "r_C", "R_Cj", "componentId"], label)?.clone();
    for k in ["mass", "inertia"] {
        if !p.contains_key(k) {
            return Err(format!("{label}: mass and inertia are required"));
        }
    }
    number(&p["mass"], label, true, false)?;
    inertia(&p["inertia"], label)?;
    p.entry("r_C").or_insert_with(|| vec3(position));
    p.entry("R_Cj").or_insert_with(|| mat(IDENTITY));
    p.entry("componentId")
        .or_insert_with(|| Value::String(String::new()));
    array(&p["r_C"], &[3], label)?;
    rotation(&p["R_Cj"], label)?;
    component_id(&p["componentId"])?;
    Ok(Value::Table(p))
}

fn geometry(raw: &Value) -> Result<(String, Table)> {
    let g = table(
        raw,
        &[
            "preset",
            "nLegs",
            "nActuators",
            "actuatorIndex",
            "armMount",
            "rotorArmIndex",
            "rotorPosition_C",
            "R_br",
            "spinSign",
            "legPosition_C",
        ],
        "geometry",
    )?;
    let preset = g.get("preset").and_then(Value::as_str).unwrap_or("QuadX");
    if g.get("preset").is_some_and(|v| !v.is_str()) {
        return Err("geometry.preset: expected string".into());
    }
    let (na, nr, radius) = match preset {
        "QuadX" => (4, 4, 0.25),
        "HexaX" => (6, 6, 0.30),
        "OctoX" => (8, 8, 0.35),
        "CoaxialX8" => (4, 8, 0.25),
        _ => return Err("geometry.preset must be QuadX, HexaX, OctoX, or CoaxialX8".into()),
    };
    let nl = integer(g.get("nLegs").unwrap_or(&int(4)), "geometry.nLegs", 0)?;
    let nc = integer(
        g.get("nActuators").unwrap_or(&int(nr)),
        "geometry.nActuators",
        1,
    )?;
    // Bound source expansion; this is a tooling limit, not a physical model limit.
    if nl > 4096 || nc > 4096 {
        return Err("geometry: at most 4096 legs/actuator channels are supported".into());
    }
    let polygon = |n: usize, r: f64, z: f64, phase: f64| {
        Value::Array(
            (0..n)
                .map(|i| {
                    let theta = 2. * std::f64::consts::PI * i as f64 / n as f64 + phase;
                    vec3([r * theta.cos(), r * theta.sin(), z])
                })
                .collect(),
        )
    };
    let mounts = g
        .get("armMount")
        .cloned()
        .unwrap_or_else(|| polygon(na, radius, 0., std::f64::consts::PI / na as f64));
    array(&mounts, &[na, 3], "geometry.armMount")?;
    let arms = g
        .get("rotorArmIndex")
        .cloned()
        .unwrap_or_else(|| Value::Array((0..nr).map(|i| int(1 + i % na)).collect()));
    let channels = g
        .get("actuatorIndex")
        .cloned()
        .unwrap_or_else(|| Value::Array((1..=nr).map(int).collect()));
    for (key, values, max) in [
        ("rotorArmIndex", &arms, na),
        ("actuatorIndex", &channels, nc),
    ] {
        array(values, &[nr], key)?;
        for v in values.as_array().unwrap() {
            if integer(v, key, 1)? > max {
                return Err(format!("geometry.{key}: index exceeds {max}"));
            }
        }
    }
    let positions = g.get("rotorPosition_C").cloned().unwrap_or_else(|| {
        Value::Array(
            (0..nr)
                .map(|i| {
                    let p = &mounts[arms[i].as_integer().unwrap() as usize - 1];
                    let dz = if preset == "CoaxialX8" {
                        if i < 4 {
                            0.025
                        } else {
                            -0.025
                        }
                    } else {
                        0.
                    };
                    vec3([
                        number(&p[0], "mount", false, false).unwrap(),
                        number(&p[1], "mount", false, false).unwrap(),
                        number(&p[2], "mount", false, false).unwrap() + dz,
                    ])
                })
                .collect(),
        )
    });
    let spins = g.get("spinSign").cloned().unwrap_or_else(|| {
        Value::Array(
            (0..nr)
                .map(|i| {
                    Value::Integer(
                        (if i % 2 == 0 { 1 } else { -1 })
                            * if preset == "CoaxialX8" && i >= 4 {
                                -1
                            } else {
                                1
                            },
                    )
                })
                .collect(),
        )
    });
    array(&spins, &[nr], "geometry.spinSign")?;
    if spins
        .as_array()
        .unwrap()
        .iter()
        .any(|x| !matches!(x.as_integer(), Some(1 | -1)))
    {
        return Err("geometry.spinSign entries must be integers +1 or -1".into());
    }
    let rotations = g
        .get("R_br")
        .cloned()
        .unwrap_or_else(|| Value::Array(vec![mat(IDENTITY); nr]));
    array(&rotations, &[nr, 3, 3], "geometry.R_br")?;
    for r in rotations.as_array().unwrap() {
        rotation(r, "geometry.R_br")?;
    }
    let legs = g
        .get("legPosition_C")
        .cloned()
        .unwrap_or_else(|| polygon(nl, 0.18, -0.15, std::f64::consts::PI / 4.));
    array(&legs, &[nl, 3], "geometry.legPosition_C")?;
    array(&positions, &[nr, 3], "geometry.rotorPosition_C")?;
    let t = [
        ("nArms", int(na)),
        ("nRotors", int(nr)),
        ("nLegs", int(nl)),
        ("nActuators", int(nc)),
        ("armMount", mounts),
        ("rotorArmIndex", arms),
        ("actuatorIndex", channels),
        ("rotorPosition_C", positions),
        ("R_br", rotations),
        ("spinSign", spins),
        ("legPosition_C", legs),
    ]
    .into_iter()
    .map(|(k, v)| (k.into(), v))
    .collect();
    Ok((preset.into(), t))
}

fn mass(raw: &Value, g: &Table) -> Result<Table> {
    let m = table(
        raw,
        &[
            "mode",
            "aggregate",
            "core",
            "arm",
            "payloads",
            "additional_parts",
        ],
        "mass",
    )?;
    let mode = m.get("mode").and_then(Value::as_str).unwrap_or("aggregate");
    if m.get("mode").is_some_and(|v| !v.is_str()) {
        return Err("mass.mode: expected string".into());
    }
    let mut out = Table::new();
    out.insert("mode".into(), Value::String(mode.into()));
    if mode == "aggregate" {
        if m.keys()
            .any(|k| !["mode", "aggregate"].contains(&k.as_str()))
        {
            return Err("mass: constituent fields are not permitted in aggregate mode".into());
        }
        let default = Value::Table(
            [
                ("mass".into(), Value::Float(1.5)),
                (
                    "inertia".into(),
                    mat([[0.02, 0., 0.], [0., 0.02, 0.], [0., 0., 0.04]]),
                ),
            ]
            .into_iter()
            .collect(),
        );
        out.insert(
            "aggregate".into(),
            part(
                m.get("aggregate").unwrap_or(&default),
                "mass.aggregate",
                [0.; 3],
            )?,
        );
    } else if mode == "assembled" {
        if m.contains_key("aggregate") || !m.contains_key("core") || !m.contains_key("arm") {
            return Err("mass: assembled mode requires core and arm, and forbids aggregate".into());
        }
        out.insert("core".into(), part(&m["core"], "mass.core", [0.; 3])?);
        let mut arm = table(
            &m["arm"],
            &["mass", "inertia", "R_Cj", "componentIds"],
            "mass.arm",
        )?
        .clone();
        let na = g["nArms"].as_integer().unwrap() as usize;
        let ids = arm.remove("componentIds").unwrap_or_else(|| {
            Value::Array(
                (1..=na)
                    .map(|i| Value::String(format!("arm.{i}")))
                    .collect(),
            )
        });
        let ids = ids
            .as_array()
            .filter(|v| v.len() == na)
            .ok_or_else(|| format!("mass.arm.componentIds: expected {na} entries"))?;
        let mut arms = Vec::new();
        for (i, id) in ids.iter().enumerate() {
            arm.insert("componentId".into(), id.clone());
            let mut p = [0.; 3];
            for (j, x) in p.iter_mut().enumerate() {
                *x = number(&g["armMount"][i][j], "mount", false, false)? / 2.;
            }
            arms.push(part(&Value::Table(arm.clone()), "mass.arm", p)?);
        }
        out.insert("arms".into(), Value::Array(arms));
        for key in ["payloads", "additional_parts"] {
            let mut parts = Vec::new();
            if let Some(v) = m.get(key) {
                for p in v
                    .as_array()
                    .ok_or_else(|| format!("mass.{key}: expected array of tables"))?
                {
                    parts.push(part(p, key, [0.; 3])?);
                }
            }
            out.insert(key.into(), Value::Array(parts));
        }
        let mut ids = BTreeSet::new();
        let parts = std::iter::once(&out["core"]).chain(
            ["arms", "payloads", "additional_parts"]
                .into_iter()
                .flat_map(|k| out[k].as_array().unwrap()),
        );
        for p in parts {
            let id = p["componentId"].as_str().unwrap();
            if !id.is_empty() && !ids.insert(id) {
                return Err("mass: duplicate nonempty componentId".into());
            }
        }
    } else {
        return Err("mass.mode must be aggregate or assembled".into());
    }
    Ok(out)
}

fn defaults(
    raw: &Value,
    fields: &[(&str, f64)],
    label: &str,
    strictly_positive: bool,
) -> Result<Table> {
    let allowed = fields.iter().map(|(k, _)| *k).collect::<Vec<_>>();
    let t = table(raw, &allowed, label)?;
    let mut out = Table::new();
    for (k, default) in fields {
        let v = t.get(*k).cloned().unwrap_or(Value::Float(*default));
        number(
            &v,
            &format!("{label}.{k}"),
            strictly_positive || *k == "legStiffness",
            !strictly_positive,
        )?;
        out.insert((*k).into(), v);
    }
    Ok(out)
}

pub fn parse(source: &str) -> Result<ResolvedFireConfig> {
    parse_at(source, None)
}

/// Select a TOML table before applying the standalone physics schema.
/// The selector uses TOML dotted-key syntax, including quoted key segments.
pub fn parse_at(source: &str, path: Option<&str>) -> Result<ResolvedFireConfig> {
    let mut raw: Value = toml::from_str(source).map_err(|e| e.to_string())?;
    if let Some(path) = path {
        let selector: Value = toml::from_str(&format!("[{path}]"))
            .map_err(|e| format!("invalid config table path {path:?}: {e}"))?;
        let mut cursor = &selector;
        let mut selected = &raw;
        loop {
            let keys = cursor
                .as_table()
                .ok_or("config table path must name a table")?;
            if keys.is_empty() {
                break;
            }
            if keys.len() != 1 {
                return Err("config table path must name exactly one table".into());
            }
            let (key, child) = keys.iter().next().unwrap();
            selected = selected
                .get(key)
                .ok_or_else(|| format!("config table {path:?} not found"))?;
            cursor = child;
        }
        if !selected.is_table() {
            return Err(format!("config table {path:?} must be a table"));
        }
        raw = selected.clone();
    }
    let root = raw.as_table().ok_or("configuration must be a table")?;
    let version = root
        .get("schema_version")
        .and_then(Value::as_integer)
        .ok_or("schema_version must be 1 or 2")?;
    let mut allowed = vec![
        "schema_version",
        "model_name",
        "geometry",
        "mass",
        "motor",
        "sensors",
        "initial",
        "ground",
    ];
    if version == 2 {
        allowed.extend(["models", "interface", "acquisition"]);
    } else if version != 1 {
        return Err("schema_version must be 1 or 2".into());
    }
    table(&raw, &allowed, "config")?;
    let acquisition = root
        .get("acquisition")
        .and_then(Value::as_str)
        .unwrap_or("sampled");
    if !["sampled", "continuous"].contains(&acquisition)
        || root
            .get("acquisition")
            .is_some_and(|v| v.as_str().is_none())
    {
        return Err("acquisition must be sampled or continuous".into());
    }
    let name = root
        .get("model_name")
        .cloned()
        .unwrap_or(Value::String("ConfiguredCopter".into()));
    let name = name.as_str().ok_or("model_name must be an identifier")?;
    if !identifier(name) || !name.starts_with(|c: char| c.is_ascii_uppercase()) {
        return Err("model_name must be an identifier starting with an uppercase letter".into());
    }
    let (preset, g) = geometry(&section(root, "geometry"))?;
    let m = mass(&section(root, "mass"), &g)?;
    let nr = g["nRotors"].as_integer().unwrap() as usize;
    let motor = resolve_motor(&section(root, "motor"), nr)?;
    let sensors = defaults(
        &section(root, "sensors"),
        &[
            ("imuSamplePeriod", 0.0025),
            ("magnetometerSamplePeriod", 0.02),
            ("gnssSamplePeriod", 0.2),
            ("barometerSamplePeriod", 0.02),
        ],
        "sensors",
        true,
    )?;
    let ground = defaults(
        &section(root, "ground"),
        &[
            ("legStiffness", 1500.),
            ("legDamping", 25.),
            ("tangentialDamping", 10.),
            ("frictionCoefficient", 0.6),
        ],
        "ground",
        false,
    )?;
    let initial = resolve_initial(&section(root, "initial"))?;
    let models = resolve_models(&section(root, "models"))?;
    let interface = resolve_interface(&section(root, "interface"), version)?;
    Ok(ResolvedFireConfig {
        schema_version: version,
        acquisition: acquisition.into(),
        model_name: name.into(),
        preset,
        geometry: g,
        mass: m,
        motor,
        sensors,
        initial,
        ground,
        models,
        interface,
    })
}

fn resolve_motor(raw: &Value, n: usize) -> Result<Table> {
    let fields = [
        ("omegaMax", 1000.),
        ("tau", 0.03),
        ("kT", 1e-5),
        ("kQ", 1.5e-7),
        ("omega_start", 0.),
    ];
    let t = table(raw, &fields.map(|(k, _)| k), "motor")?;
    let mut out = Table::new();
    for (k, default) in fields {
        let v = t.get(k).cloned().unwrap_or(Value::Float(default));
        let v = if v.is_array() {
            v
        } else {
            Value::Array(vec![v; n])
        };
        array(&v, &[n], &format!("motor.{k}"))?;
        for x in v.as_array().unwrap() {
            number(
                x,
                &format!("motor.{k}"),
                ["omegaMax", "tau", "kT"].contains(&k),
                true,
            )?;
        }
        out.insert(k.into(), v);
    }
    for i in 0..n {
        if number(&out["omega_start"][i], "motor", false, false)?
            > number(&out["omegaMax"][i], "motor", false, false)?
        {
            return Err("motor.omega_start must not exceed omegaMax".into());
        }
    }
    Ok(out)
}

fn resolve_initial(raw: &Value) -> Result<Table> {
    let t = table(
        raw,
        &["p_start", "v_start", "q_start", "omega_start"],
        "initial",
    )?;
    let mut out = Table::new();
    for (k, default) in [
        ("p_start", vec![0., 0., 1.]),
        ("v_start", vec![0.; 3]),
        ("q_start", vec![1., 0., 0., 0.]),
        ("omega_start", vec![0.; 3]),
    ] {
        let n = default.len();
        let v = t
            .get(k)
            .cloned()
            .unwrap_or_else(|| Value::Array(default.into_iter().map(Value::Float).collect()));
        array(&v, &[n], &format!("initial.{k}"))?;
        out.insert(k.into(), v);
    }
    let norm = out["q_start"]
        .as_array()
        .unwrap()
        .iter()
        .try_fold(0f64, |s, x| {
            Ok::<_, String>(s.hypot(number(x, "initial.q_start", false, false)?))
        })?;
    if norm <= 1e-12 {
        return Err("initial.q_start must be nonzero".into());
    }
    Ok(out)
}

fn resolve_models(raw: &Value) -> Result<Table> {
    let t = table(
        raw,
        &["rotor", "imu", "magnetometer", "gnss", "barometer"],
        "models",
    )?;
    if let Some(rotor) = t.get("rotor") {
        if rotor.as_str() != Some("speed_driven") {
            return Err("models.rotor: supported model is speed_driven".into());
        }
    }
    let mut out = t.clone();
    for (k, n) in [
        ("imu", 6),
        ("magnetometer", 3),
        ("gnss", 6),
        ("barometer", 4),
    ] {
        if let Some(raw) = t.get(k) {
            let v = table(raw, &["response", "tau"], &format!("models.{k}"))?;
            let response = v
                .get("response")
                .and_then(Value::as_str)
                .ok_or_else(|| format!("models.{k}.response is required"))?;
            match response {
                "ideal" if !v.contains_key("tau") => {}
                "first_order" => {
                    let tau = v
                        .get("tau")
                        .ok_or_else(|| format!("models.{k}.tau is required"))?;
                    let tau = if tau.is_array() {
                        tau.clone()
                    } else {
                        Value::Array(vec![tau.clone(); n])
                    };
                    array(&tau, &[n], &format!("models.{k}.tau"))?;
                    for x in tau.as_array().unwrap() {
                        number(x, "response tau", false, true)?;
                    }
                    out.get_mut(k)
                        .unwrap()
                        .as_table_mut()
                        .unwrap()
                        .insert("tau".into(), tau);
                }
                _ => {
                    return Err(format!(
                        "models.{k}: use ideal (without tau) or first_order"
                    ))
                }
            }
        }
    }
    Ok(out)
}

fn resolve_interface(raw: &Value, version: i64) -> Result<Table> {
    let fields = [
        ("pwm_min", 1100.),
        ("pwm_max", 1900.),
        ("actuatorSamplePeriod", 0.0025),
        ("lat0", 40.414929),
        ("lon0", -86.932387),
        ("ground_alt_wgs84", 149.),
        ("earth_radius_m", 6378137.),
    ];
    let t = table(raw, &fields.map(|(k, _)| k), "interface")?;
    if version == 1 {
        return Ok(Table::new());
    }
    let mut out = Table::new();
    for (k, d) in fields {
        let v = t.get(k).cloned().unwrap_or(Value::Float(d));
        number(
            &v,
            &format!("interface.{k}"),
            ["actuatorSamplePeriod", "earth_radius_m"].contains(&k),
            false,
        )?;
        out.insert(k.into(), v);
    }
    let get = |k| number(&out[k], k, false, false);
    if get("pwm_max")? <= get("pwm_min")? {
        return Err("interface: pwm_max must exceed pwm_min".into());
    }
    if get("lat0")?.abs() >= 90. || get("lon0")?.abs() > 180. {
        return Err("interface: invalid reference latitude/longitude".into());
    }
    Ok(out)
}
