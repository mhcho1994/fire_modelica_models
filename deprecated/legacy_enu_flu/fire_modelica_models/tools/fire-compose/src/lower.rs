//! FIRE-specific mapping into the bounded composition representation.
use crate::composition::{CompositionSpec, Instance, Modifier, Value};
use crate::config::ResolvedFireConfig;
use crate::Result;
use toml::map::Map;

fn literal(v: &toml::Value) -> Result<Value> {
    Ok(match v {
        toml::Value::Boolean(v) => Value::Bool(*v),
        toml::Value::Integer(v) => Value::Integer(*v),
        toml::Value::Float(v) => Value::Real(*v),
        toml::Value::String(v) => Value::Text(v.clone()),
        toml::Value::Array(v) => Value::Array(v.iter().map(literal).collect::<Result<_>>()?),
        _ => return Err("expected a literal, not an untyped record or datetime".into()),
    })
}
fn fields(t: &Map<String, toml::Value>) -> Result<Vec<Modifier>> {
    t.iter()
        .map(|(k, v)| Ok(Modifier::Bind(k.clone(), literal(v)?)))
        .collect()
}
fn record(class: &str, v: &toml::Value) -> Result<Value> {
    Ok(Value::Record(
        class.into(),
        fields(v.as_table().ok_or("expected record fields")?)?,
    ))
}
fn geometry(c: &ResolvedFireConfig) -> Result<Value> {
    let mut g = c.geometry.clone();
    // {} has rank one. The record supplies its own [0,3] default.
    if g["nLegs"].as_integer() == Some(0) {
        g.remove("legPosition_C");
    }
    Ok(Value::Record(
        "fire_modelica_models.Vehicles.Copter.Geometry".into(),
        fields(&g)?,
    ))
}
fn plant_modifiers(c: &ResolvedFireConfig) -> Result<Vec<Modifier>> {
    let assembled = c.mass["mode"].as_str() == Some("assembled");
    let mut m = vec![Modifier::Bind(
        "useAssembledMass".into(),
        Value::Bool(assembled),
    )];
    if assembled {
        m.push(Modifier::Bind(
            "core".into(),
            record(
                "fire_modelica_models.Physical.Mechanical.Chassis.CenterBody.RigidCenterBody",
                &c.mass["core"],
            )?,
        ));
        for (key, field, class, count) in [
            (
                "arms",
                "arms",
                "fire_modelica_models.Physical.Mechanical.Chassis.Arms.RigidArm",
                None,
            ),
            (
                "payloads",
                "payloads",
                "fire_modelica_models.Physical.Mechanical.Chassis.Payloads.FixedPayload",
                Some("nPayloads"),
            ),
            (
                "additional_parts",
                "additionalParts",
                "fire_modelica_models.Physical.Mechanical.MassProperties",
                Some("nAdditionalParts"),
            ),
        ] {
            let parts = c.mass[key].as_array().unwrap();
            if let Some(n) = count {
                m.push(Modifier::Bind(n.into(), Value::Integer(parts.len() as i64)));
            }
            if !parts.is_empty() {
                m.push(Modifier::Bind(
                    field.into(),
                    Value::Array(
                        parts
                            .iter()
                            .map(|p| record(class, p))
                            .collect::<Result<_>>()?,
                    ),
                ));
            }
        }
    } else {
        m.push(Modifier::Bind(
            "aggregate".into(),
            record(
                "fire_modelica_models.Physical.Mechanical.MassProperties",
                &c.mass["aggregate"],
            )?,
        ));
    }
    for (k, v) in &c.motor {
        let name = match k.as_str() {
            "tau" => "motorTau",
            "omega_start" => "rotorSpeed_start",
            other => other,
        };
        m.push(Modifier::Bind(name.into(), literal(v)?));
    }
    for t in [&c.sensors, &c.initial, &c.ground] {
        m.extend(fields(t)?);
    }
    if c.models.contains_key("rotor") {
        m.push(Modifier::RedeclareModel {
            name: "RotorUnit".into(),
            class: "fire_modelica_models.Systems.Propulsion.SpeedDrivenRotor".into(),
            modifiers: vec![],
        });
    }
    let mut sensors = Vec::new();
    for key in ["imu", "magnetometer", "gnss", "barometer"] {
        if let Some(t) = c.models.get(key) {
            let (class, mods) = if t["response"].as_str() == Some("first_order") {
                (
                    "FirstOrder",
                    vec![Modifier::Bind("tau".into(), literal(&t["tau"])?)],
                )
            } else {
                ("Ideal", vec![])
            };
            sensors.push(Modifier::Nested(
                key.into(),
                vec![Modifier::RedeclareModel {
                    name: "Response".into(),
                    class: format!("fire_modelica_models.Systems.Sensing.ResponseModels.{class}"),
                    modifiers: mods,
                }],
            ));
        }
    }
    if !sensors.is_empty() {
        m.push(Modifier::Nested("sensors".into(), sensors));
    }
    Ok(m)
}

pub fn compose(c: &ResolvedFireConfig, profile: &str) -> Result<CompositionSpec> {
    let plant_class = "fire_modelica_models.Vehicles.Copter.MultirotorWithSensors";
    let mut modifiers = plant_modifiers(c)?;
    modifiers.push(Modifier::Bind(
        "sampledSensors".into(),
        Value::Bool(c.acquisition == "sampled"),
    ));
    match profile {
        "plant" => {
            modifiers.insert(0, Modifier::Bind("geometry".into(), geometry(c)?));
            Ok(CompositionSpec {
                name: c.model_name.clone(),
                base: plant_class.into(),
                modifiers,
                components: vec![],
                equalities: vec![],
            })
        }
        "fastdyn" => {
            modifiers.push(Modifier::Bind(
                "pressure".into(),
                Value::Reference("ambientPressure".into()),
            ));
            modifiers.push(Modifier::Bind(
                "temperature".into(),
                Value::Reference("ambientTemperature".into()),
            ));
            modifiers.insert(
                0,
                Modifier::Bind("geometry".into(), Value::Reference("geometry".into())),
            );
            let mut boundary = vec![Modifier::Bind("geometry".into(), geometry(c)?)];
            boundary.extend(fields(&c.interface)?);
            boundary.push(Modifier::Bind(
                "sampledActuators".into(),
                Value::Bool(c.acquisition == "sampled"),
            ));
            Ok(CompositionSpec {
                name: c.model_name.clone(),
                base: "fire_modelica_models.Adapters.FastDyn.CopterInterface".into(),
                modifiers: boundary,
                components: vec![Instance {
                    class: plant_class.into(),
                    name: "plant".into(),
                    modifiers,
                }],
                equalities: [
                    ("plant.demand", "commands.demand"),
                    ("values.measurements", "plant.measurements"),
                    ("truth", "plant.truth"),
                    ("rotorSpeed", "plant.rotors.rotorSpeed"),
                ]
                .into_iter()
                .map(|(a, b)| (a.into(), b.into()))
                .collect(),
            })
        }
        _ => Err(format!(
            "unsupported profile {profile}; expected plant or fastdyn"
        )),
    }
}
