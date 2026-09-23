//! A deliberately bounded, domain-independent source composition representation.
//! No compiler IR, dynamics, or FIRE presets belong in this module.
use crate::{identifier, Result};

#[derive(Clone, Debug)]
pub enum Value {
    Bool(bool),
    Integer(i64),
    Real(f64),
    Text(String),
    Array(Vec<Value>),
    Record(String, Vec<Modifier>),
    Reference(String),
}

#[derive(Clone, Debug)]
pub enum Modifier {
    Bind(String, Value),
    Nested(String, Vec<Modifier>),
    RedeclareModel {
        name: String,
        class: String,
        modifiers: Vec<Modifier>,
    },
}

#[derive(Clone, Debug)]
pub struct Instance {
    pub class: String,
    pub name: String,
    pub modifiers: Vec<Modifier>,
}

#[derive(Clone, Debug)]
pub struct CompositionSpec {
    pub name: String,
    pub base: String,
    pub modifiers: Vec<Modifier>,
    pub components: Vec<Instance>,
    /// Declarative equality equations between references, not assignments.
    pub equalities: Vec<(String, String)>,
}

fn path(name: &str) -> Result<&str> {
    if name.split('.').all(identifier) {
        Ok(name)
    } else {
        Err(format!("invalid Modelica identifier path: {name}"))
    }
}

fn value(v: &Value) -> Result<String> {
    Ok(match v {
        Value::Bool(v) => v.to_string(),
        Value::Integer(v) => v.to_string(),
        Value::Real(v) if v.is_finite() => format!("{v:?}"),
        Value::Real(_) => return Err("non-finite Modelica literal".into()),
        Value::Text(v) => serde_json::to_string(v).map_err(|e| e.to_string())?,
        Value::Reference(v) => path(v)?.to_owned(),
        Value::Array(items) => format!(
            "{{{}}}",
            items
                .iter()
                .map(value)
                .collect::<Result<Vec<_>>>()?
                .join(",")
        ),
        Value::Record(class, fields) => format!("{}({})", path(class)?, modifiers(fields)?),
    })
}

fn modifiers(items: &[Modifier]) -> Result<String> {
    let mut names = std::collections::BTreeSet::new();
    let mut result = Vec::new();
    for item in items {
        let name = match item {
            Modifier::Bind(n, _) | Modifier::Nested(n, _) => n,
            Modifier::RedeclareModel { name, .. } => name,
        };
        path(name)?;
        if !names.insert(name) {
            return Err(format!("duplicate modifier: {name}"));
        }
        result.push(match item {
            Modifier::Bind(n, v) => format!("{n}={}", value(v)?),
            Modifier::Nested(n, m) => format!("{n}({})", modifiers(m)?),
            Modifier::RedeclareModel {
                name,
                class,
                modifiers: m,
            } => format!("redeclare model {name}={}({})", path(class)?, modifiers(m)?),
        });
    }
    Ok(result.join(",\n    "))
}

impl CompositionSpec {
    pub fn emit(&self) -> Result<String> {
        if !identifier(&self.name) {
            return Err("invalid top-level model identifier".into());
        }
        let mut text = format!(
            "within;\nmodel {}\n  extends {}(\n    {});\n",
            self.name,
            path(&self.base)?,
            modifiers(&self.modifiers)?
        );
        let mut names = std::collections::BTreeSet::new();
        for c in &self.components {
            if !identifier(&c.name) || !names.insert(&c.name) {
                return Err(format!("invalid or duplicate component name: {}", c.name));
            }
            text.push_str(&format!(
                "  {} {}(\n    {});\n",
                path(&c.class)?,
                c.name,
                modifiers(&c.modifiers)?
            ));
        }
        if !self.equalities.is_empty() {
            text.push_str("equation\n");
        }
        for (left, right) in &self.equalities {
            text.push_str(&format!("  {} = {};\n", path(left)?, path(right)?));
        }
        text.push_str(&format!("end {};\n", self.name));
        Ok(text)
    }
}
