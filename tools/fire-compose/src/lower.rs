//! FIRE-specific mapping into the bounded composition representation.
use crate::composition::{CompositionSpec, Instance, Modifier, Prefix, Value};
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
fn mass_record(class: &str, v: &toml::Value) -> Result<Value> {
    // Part identities are validated configuration/manifest metadata, not physics inputs.
    let modifiers = v
        .as_table()
        .ok_or("expected mass record fields")?
        .iter()
        .filter(|(name, _)| name.as_str() != "componentId")
        .map(|(name, value)| Ok(Modifier::Bind(name.clone(), literal(value)?)))
        .collect::<Result<_>>()?;
    Ok(Value::Record(class.into(), modifiers))
}
fn geometry(c: &ResolvedFireConfig) -> Result<Value> {
    Ok(Value::Record(
        "fire_modelica_models.Vehicles.Copter.Geometry".into(),
        fields(&c.geometry)?,
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
            mass_record(
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
                            .map(|p| mass_record(class, p))
                            .collect::<Result<_>>()?,
                    ),
                ));
            }
        }
    } else {
        m.push(Modifier::Bind(
            "aggregate".into(),
            mass_record(
                "fire_modelica_models.Physical.Mechanical.MassProperties",
                &c.mass["aggregate"],
            )?,
        ));
    }
    m.extend(fields(&c.initial)?);
    m.push(Modifier::Bind(
        "landingGear".into(),
        Value::Record(
            "fire_modelica_models.Physical.Mechanical.Chassis.LandingGear.Parameters".into(),
            fields(&c.landing_gear)?,
        ),
    ));
    Ok(m)
}

fn instance(class: &str, name: &str, dimensions: Vec<usize>, modifiers: Vec<Modifier>) -> Instance {
    Instance {
        class: class.into(),
        name: name.into(),
        dimensions,
        modifiers,
        prefix: Prefix::Component,
        binding: None,
    }
}
fn reference(name: &str) -> Value {
    Value::Reference(name.into())
}

pub fn compose(c: &ResolvedFireConfig, profile: &str) -> Result<CompositionSpec> {
    let profile = crate::canonical_profile(profile)?;
    let vio = profile == "vio";
    let dynamic = c.electromechanical();
    let three_phase = c.three_phase();
    let p2 = dynamic && !three_phase;
    let mut connections = Vec::new();
    let nr = c.geometry["nRotors"].as_integer().unwrap() as usize;
    let nc = c.geometry["nActuators"].as_integer().unwrap() as usize;
    let plant_class = "fire_modelica_models.Vehicles.Copter.MultirotorWithSensors";
    let mut physical = plant_modifiers(c)?;
    let mut components = Vec::new();
    let (base, modifiers, command) = if vio {
        physical.insert(0, Modifier::Bind("geometry".into(), reference("geometry")));
        physical.push(Modifier::Bind("pressure".into(), Value::Real(101325.0)));
        physical.push(Modifier::Bind("temperature".into(), Value::Real(288.15)));
        components.push(instance(plant_class, "plant", vec![], physical));
        let mut boundary = vec![Modifier::Bind("geometry".into(), geometry(c)?)];
        boundary.extend(fields(&c.interface)?);
        (
            Some("fire_modelica_models.Exports.FMU.CopterInterface".into()),
            boundary,
            "commands.normalizedCommand",
        )
    } else {
        let mut layout = instance(
            "fire_modelica_models.Vehicles.Copter.Geometry",
            "geometry",
            vec![],
            vec![],
        );
        layout.prefix = Prefix::Parameter;
        layout.binding = Some(geometry(c)?);
        components.push(layout);
        physical.insert(0, Modifier::Bind("geometry".into(), reference("geometry")));
        // Keep environmental inputs public; internally computed rotor wrenches are not inputs.
        for (name, class, default) in [
            (
                "gravity_w",
                "Modelica.Units.SI.Acceleration",
                vec![0., 0., 9.80665],
            ),
            ("wind_w", "Modelica.Units.SI.Velocity", vec![0.; 3]),
            ("externalForce_b", "Modelica.Units.SI.Force", vec![0.; 3]),
            ("externalMoment_b", "Modelica.Units.SI.Torque", vec![0.; 3]),
            (
                "magneticField_w",
                "Modelica.Units.SI.MagneticFluxDensity",
                vec![0., 20e-6, 45e-6],
            ),
            ("pressure", "Modelica.Units.SI.Pressure", vec![101325.]),
            ("temperature", "Modelica.Units.SI.Temperature", vec![288.15]),
        ] {
            let scalar = default.len() == 1;
            let dimensions = if scalar { vec![] } else { vec![default.len()] };
            let mut input = instance(class, name, dimensions, vec![]);
            input.prefix = Prefix::Input;
            input.binding = Some(if scalar {
                Value::Real(default[0])
            } else {
                Value::Array(default.into_iter().map(Value::Real).collect())
            });
            components.push(input);
            physical.push(Modifier::Bind(name.into(), reference(name)));
        }
        components.push(instance(plant_class, "plant", vec![], physical));
        for (name, class, dimensions) in [
            (
                "truth",
                "fire_modelica_models.Interfaces.VehicleState",
                vec![],
            ),
            (
                "measurements",
                "fire_modelica_models.Interfaces.SensorMeasurements",
                vec![],
            ),
            ("force_b", "Modelica.Units.SI.Force", vec![3]),
            ("moment_b", "Modelica.Units.SI.Torque", vec![3]),
        ] {
            let mut output = instance(class, name, dimensions, vec![]);
            output.prefix = Prefix::Output;
            components.push(output);
        }
        let mut input = instance(
            "Modelica.Units.SI.PerUnit",
            "normalizedCommand",
            vec![nc],
            vec![],
        );
        input.prefix = Prefix::Input;
        components.push(input);
        let mut speed = instance(
            "Modelica.Units.SI.AngularVelocity",
            "rotorSpeed",
            vec![nr],
            vec![],
        );
        speed.prefix = Prefix::Output;
        components.push(speed);
        (None, vec![], "normalizedCommand")
    };
    components.push(instance(c.esc_class(), "escs", vec![nr], fields(&c.esc)?));
    let mut motor_fields = fields(&c.motor)?;
    if dynamic {
        motor_fields.push(Modifier::Bind("JLoad".into(), literal(&c.rotor["J"])?));
    }
    components.push(instance(c.motor_class(), "motors", vec![nr], motor_fields));
    let mut rotor_fields = c.rotor.clone();
    rotor_fields.remove("J");
    let mut blades = fields(&rotor_fields)?;
    blades.push(Modifier::Bind(
        "spinSign".into(),
        reference("geometry.spinSign"),
    ));
    components.push(instance(
        "fire_modelica_models.Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade",
        "rotors",
        vec![nr],
        blades,
    ));
    let mapped = c.geometry["actuatorIndex"]
        .as_array()
        .unwrap()
        .iter()
        .map(|i| reference(&format!("{command}[{}]", i.as_integer().unwrap())))
        .collect();
    let mut equalities = vec![
        ("escs.escCommand".into(), Value::Array(mapped)),
        ("rotors.omega".into(), reference("motors.omega")),
        ("rotorSpeed".into(), reference("motors.omega")),
        (
            "rotors.airVelocity_r".into(),
            reference("plant.rotorAirVelocity_r"),
        ),
        (
            "rotors.omegaBody_r".into(),
            reference("plant.rotorOmegaBody_r"),
        ),
        ("plant.rotorForce_r".into(), reference("rotors.force_r")),
        (
            "plant.rotorMoment_r".into(),
            reference(if dynamic {
                "reactions.moment_r"
            } else {
                "rotors.moment_r"
            }),
        ),
    ];
    if c.voltage_driven() {
        let mut voltage = instance("Modelica.Units.SI.Voltage", "dcVoltage", vec![], vec![]);
        voltage.prefix = if vio {
            Prefix::Parameter
        } else {
            Prefix::Input
        };
        voltage.binding = Some(literal(&c.power["dcVoltage"])?);
        components.push(voltage);
        if !three_phase {
            equalities.push((
                "escs.dcVoltage".into(),
                Value::Array(vec![reference("dcVoltage"); nr]),
            ));
        }
        if !dynamic {
            equalities.push((
                "motors.equivalentVoltage".into(),
                reference("escs.equivalentVoltage"),
            ));
        }
    } else {
        equalities.push(("motors.demand".into(), reference("escs.normalizedCommand")));
    }
    if dynamic {
        components.push(instance(
            "fire_modelica_models.Systems.Propulsion.HubReaction",
            "reactions",
            vec![nr],
            vec![
                Modifier::Bind("J".into(), literal(&c.motor["J"])?),
                Modifier::Bind("JLoad".into(), literal(&c.rotor["J"])?),
                Modifier::Bind("spinSign".into(), reference("geometry.spinSign")),
            ],
        ));
        if p2 {
            connections.push(("escs.p".into(), "motors.p".into()));
            connections.push(("escs.n".into(), "motors.n".into()));
            equalities.extend([
                ("motors.currentZero".into(), reference("escs.coast")),
                (
                    "escs.openCircuitVoltage".into(),
                    reference("motors.backEMF"),
                ),
            ]);
            let mut current = instance(
                "Modelica.Units.SI.Current",
                "motorCurrent",
                vec![nr],
                vec![],
            );
            current.prefix = Prefix::Output;
            components.push(current);
            equalities.push(("motorCurrent".into(), reference("motors.current")));
        }
        equalities.extend([
            ("motors.loadTorque".into(), reference("rotors.loadTorque")),
            (
                "reactions.aerodynamicMoment_r".into(),
                reference("rotors.moment_r"),
            ),
            (
                "reactions.acceleration".into(),
                reference("motors.acceleration"),
            ),
        ]);
        for (name, class, source) in [
            ("dcCurrent", "Current", "escs.dcCurrent"),
            (
                "electromagneticTorque",
                "Torque",
                "motors.electromagneticTorque",
            ),
            ("loadTorque", "Torque", "rotors.loadTorque"),
            ("electricalPower", "Power", "escs.electricalPower"),
            ("copperLoss", "Power", "motors.copperLoss"),
            ("frictionLoss", "Power", "motors.frictionLoss"),
            ("loadPower", "Power", "motors.loadPower"),
            ("storedEnergy", "Energy", "motors.storedEnergy"),
        ] {
            let mut diagnostic = instance(
                &format!("Modelica.Units.SI.{class}"),
                name,
                vec![nr],
                vec![],
            );
            diagnostic.prefix = Prefix::Output;
            components.push(diagnostic);
            equalities.push((name.into(), reference(source)));
        }
    }
    if three_phase {
        components.push(instance(
            "fire_modelica_models.Systems.Power.DCLink",
            "bus",
            vec![],
            vec![
                Modifier::Bind(
                    "sourceResistance".into(),
                    literal(&c.power["sourceResistance"])?,
                ),
                Modifier::Bind(
                    "dcLinkCapacitance".into(),
                    literal(&c.power["dcLinkCapacitance"])?,
                ),
                Modifier::Bind("voltage_start".into(), literal(&c.power["dcVoltage"])?),
            ],
        ));
        equalities.push(("bus.sourceVoltage".into(), reference("dcVoltage")));
        equalities.push((
            "escs.electricalAngle".into(),
            reference("motors.electricalAngle"),
        ));
        for i in 1..=nr {
            connections.push(("bus.p".into(), format!("escs[{i}].dc_p")));
            connections.push(("bus.n".into(), format!("escs[{i}].dc_n")));
            connections.push((format!("escs[{i}].phase"), format!("motors[{i}].phase")));
        }
        for (name, class, dims, source) in [
            (
                "phaseCurrent",
                "Current",
                vec![nr, 3],
                "motors.phaseCurrent",
            ),
            ("backEMF", "Voltage", vec![nr, 3], "motors.backEMF"),
            (
                "electricalAngle",
                "Angle",
                vec![nr],
                "motors.electricalAngle",
            ),
            ("escLoss", "Power", vec![nr], "escs.lossPower"),
            ("busVoltage", "Voltage", vec![], "bus.voltage"),
            ("totalDcCurrent", "Current", vec![], "bus.loadCurrent"),
            ("sourceCurrent", "Current", vec![], "bus.sourceCurrent"),
            ("sourcePower", "Power", vec![], "bus.sourcePower"),
            ("busLoss", "Power", vec![], "bus.lossPower"),
            ("busStoredEnergy", "Energy", vec![], "bus.storedEnergy"),
        ] {
            let mut diagnostic =
                instance(&format!("Modelica.Units.SI.{class}"), name, dims, vec![]);
            diagnostic.prefix = Prefix::Output;
            components.push(diagnostic);
            equalities.push((name.into(), reference(source)));
        }
    }
    if vio {
        equalities.push((
            "values.measurements".into(),
            reference("plant.measurements"),
        ));
    } else {
        for name in ["measurements", "force_b", "moment_b"] {
            equalities.push((name.into(), reference(&format!("plant.{name}"))));
        }
    }
    equalities.push(("truth".into(), reference("plant.truth")));
    Ok(CompositionSpec {
        name: c.model_name.clone(),
        base,
        modifiers,
        components,
        equalities,
        connections,
    })
}
