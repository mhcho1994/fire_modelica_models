//! Deterministic artifacts and provenance, separate from compilation status.
use crate::{
    config::ResolvedFireConfig, lower, Result, GENERATOR, HEADER, LEGACY_GENERATOR, LEGACY_HEADER,
};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    fs,
    path::{Component, Path, PathBuf},
    process::Command,
};

pub fn hash(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
pub fn library_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("FIRE library root")
}

fn resolved(path: &Path) -> Result<PathBuf> {
    let absolute = if path.is_absolute() {
        path.to_owned()
    } else {
        std::env::current_dir()
            .map_err(|e| e.to_string())?
            .join(path)
    };
    let mut result = PathBuf::new();
    for c in absolute.components() {
        match c {
            Component::ParentDir => {
                result.pop();
            }
            Component::CurDir => {}
            _ => {
                result.push(c);
                if fs::symlink_metadata(&result).is_ok() {
                    result = result.canonicalize().map_err(|e| e.to_string())?;
                }
            }
        }
    }
    Ok(result)
}
fn source_files(dir: &Path, root: &Path, files: &mut Vec<PathBuf>) -> Result<()> {
    for item in fs::read_dir(dir).map_err(|e| e.to_string())? {
        let path = item.map_err(|e| e.to_string())?.path();
        if path.is_symlink() {
            continue;
        }
        if path.is_dir() {
            if !["build", "target", ".git", "__pycache__", "tools"]
                .contains(&path.file_name().unwrap().to_string_lossy().as_ref())
            {
                source_files(&path, root, files)?;
            }
        } else if path.extension().is_some_and(|v| v == "mo")
            || path.file_name().is_some_and(|v| v == "package.order")
        {
            files.push(path.strip_prefix(root).unwrap().to_owned());
        }
    }
    Ok(())
}
pub fn source_identity(root: &Path) -> Result<Value> {
    let mut paths = Vec::new();
    source_files(root, root, &mut paths)?;
    paths.sort();
    let mut digest = Sha256::new();
    let mut files = serde_json::Map::new();
    for p in paths {
        let bytes = fs::read(root.join(&p)).map_err(|e| e.to_string())?;
        let name = p.to_string_lossy().replace('\\', "/");
        digest.update(name.as_bytes());
        digest.update([0]);
        digest.update(&bytes);
        digest.update([0]);
        files.insert(name, Value::String(hash(&bytes)));
    }
    let revision = Command::new("git")
        .args(["-C"])
        .arg(root)
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|r| r.status.success())
        .map(|r| String::from_utf8_lossy(&r.stdout).trim().to_owned());
    Ok(json!({"sha256":format!("{:x}",digest.finalize()),"files":files,"git_revision":revision}))
}

pub fn emit(
    config: &ResolvedFireConfig,
    raw: &str,
    profile: &str,
    root: &Path,
    output: &Path,
) -> Result<(PathBuf, PathBuf)> {
    let source = format!("{HEADER}{}", lower::compose(config, profile)?.emit()?);
    let root = root.canonicalize().map_err(|e| e.to_string())?;
    let output = resolved(output)?;
    if output.starts_with(&root) && !output.starts_with(root.join("build")) {
        return Err("Output directory must be inside ROOT/build or outside the source root".into());
    }
    let model = output.join(format!("{}.mo", config.model_name));
    let manifest = output.join(format!("{}.manifest.json", config.model_name));
    for p in [&model, &manifest] {
        if p.is_symlink() {
            return Err(format!("Refusing symlink output file: {}", p.display()));
        }
    }
    if model.exists() {
        let previous_source = fs::read_to_string(&model).map_err(|e| e.to_string())?;
        if !previous_source.starts_with(HEADER) && !previous_source.starts_with(LEGACY_HEADER) {
            return Err(format!(
                "Refusing to overwrite non-generated Modelica file: {}",
                model.display()
            ));
        }
    }
    let mut previous = None;
    if manifest.exists() {
        let prior: Value = serde_json::from_slice(&fs::read(&manifest).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        if prior["generator_id"] != GENERATOR && prior["generator_id"] != LEGACY_GENERATOR {
            return Err("Refusing to overwrite non-generated manifest".into());
        }
        previous = Some(prior);
    }
    let mut manifest_value = json!({
        "generator_id":GENERATOR,"generator_version":env!("CARGO_PKG_VERSION"),"manifest_version":1,
        "schema_version":config.schema_version,"profile":profile,"model":config.model_name,
        "config_sha256":hash(raw.as_bytes()),"config":config,"source":source_identity(&root)?,
        "generated_model_sha256":hash(source.as_bytes()),
        "interface":{"nRotors":config.geometry["nRotors"],"nActuators":config.geometry["nActuators"],
                     "actuatorIndex":config.geometry["actuatorIndex"]},
        "event_requirements":{"sampled_sensors":config.acquisition=="sampled","sampled_actuators":profile=="fastdyn" && config.acquisition=="sampled","ground_contact":config.geometry["nLegs"].as_integer().unwrap()>0},
        "verification":{"configuration":"passed","modelica_check":"not_run","fmi_generation":"not_run","fmu_simulation":"not_run","firmware":"not_run"}
    });
    if let Some(prior) = previous {
        if ["config_sha256", "generated_model_sha256", "profile"]
            .iter()
            .all(|key| prior[*key] == manifest_value[*key])
            && prior["source"]["sha256"] == manifest_value["source"]["sha256"]
        {
            if prior["verification"].is_object() {
                manifest_value["verification"] = prior["verification"].clone();
            }
            if prior["compiler"].is_object() {
                manifest_value["compiler"] = prior["compiler"].clone();
            }
        }
    }
    fs::create_dir_all(&output).map_err(|e| e.to_string())?;
    write_if_changed(&model, source.as_bytes())?;
    let text = serde_json::to_string_pretty(&manifest_value).map_err(|e| e.to_string())? + "\n";
    write_if_changed(&manifest, text.as_bytes())?;
    Ok((model, manifest))
}

fn write_if_changed(path: &Path, bytes: &[u8]) -> Result<()> {
    if fs::read(path).ok().as_deref() != Some(bytes) {
        fs::write(path, bytes).map_err(|e| e.to_string())?;
    }
    Ok(())
}

/// Stage the declared package tree under its Modelica name. Repository tooling
/// and undeclared compatibility directories are not Modelica source roots.
pub fn stage_library(root: &Path, output: &Path) -> Result<PathBuf> {
    let sources = output.join("sources");
    let staged = sources.join("fire_modelica_models");
    if sources.is_symlink() || staged.is_symlink() {
        return Err("Refusing symlink staged source directories".into());
    }
    let marker = staged.join(".fire-compose");
    if staged.exists() && fs::read_to_string(&marker).ok().as_deref() != Some(GENERATOR) {
        return Err("Refusing to overwrite an unowned staged library".into());
    }
    fn copy_package(src: &Path, dst: &Path) -> Result<()> {
        fs::create_dir_all(dst).map_err(|e| e.to_string())?;
        for name in ["package.mo", "package.order"] {
            if src.join(name).exists() {
                write_if_changed(
                    &dst.join(name),
                    &fs::read(src.join(name)).map_err(|e| e.to_string())?,
                )?;
            }
        }
        let order = fs::read_to_string(src.join("package.order")).map_err(|e| e.to_string())?;
        for name in order.lines().map(str::trim).filter(|s| !s.is_empty()) {
            if !crate::identifier(name) {
                return Err(format!("invalid package.order entry: {name}"));
            }
            if src.join(name).join("package.mo").exists() {
                copy_package(&src.join(name), &dst.join(name))?;
            } else if src.join(format!("{name}.mo")).exists() {
                let file = format!("{name}.mo");
                write_if_changed(
                    &dst.join(&file),
                    &fs::read(src.join(file)).map_err(|e| e.to_string())?,
                )?;
            }
        }
        Ok(())
    }
    fs::create_dir_all(&sources).map_err(|e| e.to_string())?;
    let work = tempfile::tempdir_in(&sources).map_err(|e| e.to_string())?;
    let fresh = work.path().join("fire_modelica_models");
    copy_package(root, &fresh)?;
    fs::write(fresh.join(".fire-compose"), GENERATOR).map_err(|e| e.to_string())?;
    // Replace only a directory owned by this generator, so removed classes do
    // not survive a subsequent composition and nested symlinks are not followed.
    let old = work.path().join("previous");
    if staged.exists() {
        fs::rename(&staged, &old).map_err(|e| e.to_string())?;
    }
    if let Err(error) = fs::rename(&fresh, &staged) {
        if old.exists() {
            let _ = fs::rename(&old, &staged);
        }
        return Err(error.to_string());
    }
    // Retire the previous namespace only when its tree belongs to this generator.
    let legacy = sources.join("FIRE_Modelica");
    if !legacy.is_symlink()
        && fs::read_to_string(legacy.join(".fire-compose"))
            .ok()
            .as_deref()
            == Some(LEGACY_GENERATOR)
    {
        fs::rename(&legacy, work.path().join("previous-namespace")).map_err(|e| e.to_string())?;
    }
    Ok(staged)
}
