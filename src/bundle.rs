use crate::mujoco::ClosedModel;
use crate::mujoco::Resource;
use phoxal::artifact::bundle::{
    AdmittedBundle, BundleComponent, BundleInstance, BundleManifest, BundleModelAssets,
    BundleSimulation,
};
use phoxal::artifact::document::{CapabilityDeclaration, ComponentDocument, ComponentModel};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Component;
use std::path::Path;
use std::path::PathBuf;

pub(super) const MODEL_ASSET_PREFIX: &str = "assets/";

pub(super) struct BundleFacts {
    pub(super) root: PathBuf,
    pub(super) robot_id: String,
    pub(super) instances: BTreeMap<String, BundleInstance>,
    pub(super) admitted: AdmittedBundle,
    pub(super) components: BTreeMap<String, BundleComponent>,
    component_sources: BTreeMap<String, String>,
    pub(super) model: Option<BundleModelAssets>,
    pub(super) simulation: Option<BundleSimulation>,
}

impl BundleFacts {
    pub(super) fn load(path: &Path) -> Result<Self, String> {
        let root = path
            .canonicalize()
            .map_err(|error| format!("cannot canonicalize bundle {}: {error}", path.display()))?;
        if !root.is_dir() {
            return Err(format!("bundle {} is not a directory", root.display()));
        }
        let manifest_bytes = read_bounded_regular(&root.join("manifest.json"), 16 * 1024 * 1024)?;
        let manifest: BundleManifest = serde_json::from_slice(&manifest_bytes)
            .map_err(|error| format!("cannot parse manifest.json: {error}"))?;
        let admitted = AdmittedBundle::validate(manifest)
            .map_err(|message| format!("invalid runtime bundle: {message}"))?;
        let facts = Self {
            root,
            robot_id: admitted.robot_id.clone(),
            instances: admitted.instances.clone(),
            components: admitted.components.clone(),
            component_sources: admitted.component_sources.clone(),
            model: admitted.model.clone(),
            simulation: admitted.simulation.clone(),
            admitted,
        };
        if facts.model.is_some() {
            facts.root_closed_model()?;
        }
        Ok(facts)
    }

    /// The compiled runtime record selected for this instance.
    pub(super) fn runtime_record(
        &self,
        instance: &str,
    ) -> Option<&phoxal::artifact::RuntimeRecord> {
        self.admitted.instance_runtime(instance)
    }

    pub(super) fn root_closed_model(&self) -> Result<ClosedModel, String> {
        let model = self
            .model
            .as_ref()
            .ok_or_else(|| "bundle has no robot model".to_owned())?;
        let entry = model
            .entry
            .strip_prefix(MODEL_ASSET_PREFIX)
            .ok_or_else(|| format!("model entry {} is outside assets/", model.entry))?;
        validate_relative_path(entry, "model entry")?;
        if !model
            .resources
            .iter()
            .any(|resource| resource == &model.entry)
        {
            return Err(format!("bundle model entry {} is not staged", model.entry));
        }
        // The assets root itself is validated from the canonical bundle
        // root: a symlinked assets directory never reaches the per-file
        // checks below.
        directory_under(&self.root, Path::new("assets"), "model assets")?;
        let mut names = BTreeSet::new();
        let mut resources = Vec::new();
        for resource in &model.resources {
            let name = resource
                .strip_prefix(MODEL_ASSET_PREFIX)
                .ok_or_else(|| format!("model resource {resource} is outside assets/"))?;
            validate_relative_path(name, "model resource")?;
            if !names.insert(name.to_owned()) {
                return Err(format!("bundle model resource {name:?} is duplicated"));
            }
            let path = regular_file_under(&self.root.join("assets"), Path::new(name), resource)?;
            let bytes = fs::read(path)
                .map_err(|error| format!("cannot read model resource {resource}: {error}"))?;
            resources
                .push(Resource::new(name.to_owned(), bytes).map_err(|error| error.to_string())?);
        }
        ClosedModel::new(entry, resources).map_err(|error| error.to_string())
    }

    pub(super) fn component_closed_model(
        &self,
        component: &BundleComponent,
        entry: &Path,
    ) -> Result<ClosedModel, String> {
        let entry = entry.to_str().ok_or_else(|| {
            format!(
                "component {} model entry is not valid UTF-8",
                component.instance
            )
        })?;
        validate_relative_path(entry, "component model entry")?;
        let relative = self
            .component_sources
            .get(&component.instance)
            .ok_or_else(|| {
                format!(
                    "component {} has no staged model source",
                    component.instance
                )
            })?;
        validate_relative_path(relative, "component source")?;
        // Every ancestor below the canonical bundle root is validated: a
        // symlinked directory component never reaches the final inspection.
        let source_root = directory_under(
            &self.root,
            Path::new(relative),
            &format!("component {} source", component.instance),
        )?;
        let mut resources = Vec::new();
        collect_resources(&source_root, &source_root, &mut resources)?;
        if !resources.iter().any(|resource| resource.name() == entry) {
            return Err(format!(
                "component {} source has no {entry}",
                component.instance
            ));
        }
        ClosedModel::new(entry, resources).map_err(|error| error.to_string())
    }
}

fn collect_resources(
    root: &Path,
    directory: &Path,
    resources: &mut Vec<Resource>,
) -> Result<(), String> {
    for entry in fs::read_dir(directory).map_err(|error| {
        format!(
            "cannot read component source {}: {error}",
            directory.display()
        )
    })? {
        let entry = entry.map_err(|error| error.to_string())?;
        let path = entry.path();
        let metadata = fs::symlink_metadata(&path).map_err(|error| {
            format!(
                "cannot inspect component resource {}: {error}",
                path.display()
            )
        })?;
        if metadata.file_type().is_symlink() {
            return Err(format!(
                "component resource {} is a symbolic link",
                path.display()
            ));
        }
        if metadata.is_dir() {
            collect_resources(root, &path, resources)?;
        } else if metadata.is_file() {
            let relative = path.strip_prefix(root).map_err(|error| error.to_string())?;
            let name = relative.to_str().ok_or_else(|| {
                format!("component resource {} is not valid UTF-8", path.display())
            })?;
            let bytes = fs::read(&path).map_err(|error| error.to_string())?;
            resources
                .push(Resource::new(name.to_owned(), bytes).map_err(|error| error.to_string())?);
        }
    }
    Ok(())
}

pub(super) fn component_definition(
    document: &ComponentDocument,
) -> (
    &ComponentModel,
    &std::collections::BTreeMap<String, CapabilityDeclaration>,
) {
    let ComponentDocument::V0 {
        model,
        capabilities,
        ..
    } = document;
    (model, capabilities)
}

pub(super) fn regular_file(path: &Path, display: &str) -> Result<(), String> {
    let metadata = fs::symlink_metadata(path)
        .map_err(|error| format!("cannot inspect bundle file {display}: {error}"))?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(format!(
            "bundle file {display} is not a regular non-symlink file"
        ));
    }
    Ok(())
}

/// Reads one file that must be regular and non-symbolic, bounded in size.
pub(super) fn read_bounded_regular(path: &Path, limit: u64) -> Result<Vec<u8>, String> {
    let metadata = fs::symlink_metadata(path)
        .map_err(|error| format!("cannot inspect {}: {error}", path.display()))?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(format!("{} is not a regular file", path.display()));
    }
    if metadata.len() > limit {
        return Err(format!(
            "{} is {} bytes, exceeding the {}-byte limit",
            path.display(),
            metadata.len(),
            limit
        ));
    }
    fs::read(path).map_err(|error| format!("cannot read {}: {error}", path.display()))
}

/// Resolves one relative directory below the canonical bundle root,
/// rejecting symbolic links in every component of the path.
fn directory_under(root: &Path, relative: &Path, label: &str) -> Result<PathBuf, String> {
    let mut path = root.to_owned();
    for component in relative.components() {
        let std::path::Component::Normal(name) = component else {
            return Err(format!(
                "{label} path {} is not a normalized relative path",
                relative.display()
            ));
        };
        path.push(name);
        let metadata = fs::symlink_metadata(&path)
            .map_err(|error| format!("cannot inspect {label} {}: {error}", path.display()))?;
        if metadata.file_type().is_symlink() {
            return Err(format!(
                "{label} path {} traverses a symbolic link",
                path.display()
            ));
        }
        if !metadata.is_dir() {
            return Err(format!("{label} {} is not a directory", path.display()));
        }
    }
    Ok(path)
}
pub(super) fn regular_file_under(
    root: &Path,
    relative: &Path,
    display: &str,
) -> Result<PathBuf, String> {
    let mut path = root.to_owned();
    let components = relative.components().collect::<Vec<_>>();
    if components.is_empty() {
        return Err(format!("bundle file {display} has an empty relative path"));
    }
    for (index, component) in components.iter().enumerate() {
        let Component::Normal(name) = component else {
            return Err(format!(
                "bundle file {display} is not a normalized relative path"
            ));
        };
        path.push(name);
        let metadata = fs::symlink_metadata(&path)
            .map_err(|error| format!("cannot inspect bundle file {display}: {error}"))?;
        if metadata.file_type().is_symlink() {
            return Err(format!("bundle file {display} traverses a symbolic link"));
        }
        let is_last = index + 1 == components.len();
        if is_last {
            if !metadata.is_file() {
                return Err(format!(
                    "bundle file {display} is not a regular non-symlink file"
                ));
            }
        } else if !metadata.is_dir() {
            return Err(format!(
                "bundle file {display} traverses a non-directory component"
            ));
        }
    }
    Ok(path)
}

pub(super) fn validate_relative_path(value: &str, field: &str) -> Result<(), String> {
    let path = Path::new(value);
    if value.is_empty()
        || path.is_absolute()
        || value.contains('\\')
        || path.components().any(|component| {
            matches!(
                component,
                Component::CurDir
                    | Component::ParentDir
                    | Component::RootDir
                    | Component::Prefix(_)
            )
        })
    {
        return Err(format!(
            "{field} path {value:?} is not normalized and relative"
        ));
    }
    Ok(())
}

#[cfg(test)]
mod confinement_tests {
    use super::*;
    use phoxal::artifact::bundle::{AdmittedBundle, BundleArtifactRecord};
    use phoxal::artifact::{RUNTIME_RECORD, RuntimeRecord};
    use std::fs;

    const ID: &str = "fixture-runtime";

    fn artifact_record() -> phoxal::artifact::bundle::BundleArtifactRecord {
        BundleArtifactRecord {
            id: ID.to_owned(),
            path: format!("bin/{ID}"),
            provenance: None,
            runtime: RuntimeRecord::V0 {
                record: RUNTIME_RECORD.to_owned(),
                conversions: Vec::new(),
                period_ms: 20,
                timeout_ms: 100,
                init_timeout_ms: 1_000,
                config_schema: serde_json::json!({"type": "object"}),
                inputs: Vec::new(),
                outputs: Vec::new(),
            },
            descriptors: Vec::new(),
        }
    }

    fn manifest_with(model: Option<serde_json::Value>) -> serde_json::Value {
        serde_json::json!({
            "schema": "phoxal/bundle/v0",
            "robot_id": "confinement-fixture",
            "target": phoxal::artifact::bundle::host_execution_target(),
            "supervisor": {"path": "bin/supervisor"},
            "artifacts": [serde_json::to_value(artifact_record()).expect("artifact encodes")],
            "instances": [
                {"id": "brain", "role": "brain", "artifact": ID},
            ],
            "connections": [],
            "components": [],
            "component_sources": {},
            "model": model,
        })
    }

    /// Writes one minimal valid bundle and returns its root.
    fn write_bundle(root: &Path, model: Option<serde_json::Value>) {
        fs::create_dir_all(root.join("bin")).expect("bin dir");
        fs::write(root.join("bin").join(ID), b"executable").expect("artifact");
        fs::write(root.join("bin/supervisor"), b"supervisor").expect("supervisor");
        if let Some(model) = &model {
            let entry = model["entry"].as_str().expect("entry").to_owned();
            let entry_path = root.join(&entry);
            fs::create_dir_all(entry_path.parent().expect("entry parent")).expect("assets dir");
            fs::write(&entry_path, "<model/>").expect("model entry");
            for resource in model["resources"].as_array().expect("resources") {
                let path = root.join(resource.as_str().expect("resource"));
                if path != entry_path {
                    fs::create_dir_all(path.parent().expect("resource parent"))
                        .expect("resource dir");
                    fs::write(&path, b"resource").expect("resource");
                }
            }
        }
        let manifest = manifest_with(model);
        AdmittedBundle::validate(
            serde_json::from_value(manifest.clone()).expect("manifest decodes"),
        )
        .expect("fixture manifest admits");
        fs::write(
            root.join("manifest.json"),
            serde_json::to_vec(&manifest).expect("manifest encodes"),
        )
        .expect("manifest");
    }

    #[test]
    fn a_symlinked_manifest_is_refused() {
        let guard = tempfile::tempdir().expect("bundle tempdir");
        let real = guard.path().join("real");
        write_bundle(&real, None);
        let linked = guard.path().join("linked");
        fs::create_dir_all(&linked).expect("linked dir");
        std::os::unix::fs::symlink(real.join("manifest.json"), linked.join("manifest.json"))
            .expect("link manifest");
        // The linked root's manifest is itself a symlink.
        let error = BundleFacts::load(&linked)
            .err()
            .expect("a symlinked manifest is refused");
        assert!(
            error.contains("is not a regular file"),
            "unexpected refusal: {error}"
        );
    }

    #[test]
    fn a_symlinked_assets_root_is_refused() {
        let guard = tempfile::tempdir().expect("bundle tempdir");
        let root = guard.path().join("bundle");
        let model = serde_json::json!({
            "entry": "assets/model.xml",
            "resources": ["assets/model.xml"],
        });
        write_bundle(&root, Some(model));
        // Replace the assets directory with a symlink to an outside tree.
        let outside = guard.path().join("outside-assets");
        fs::create_dir_all(&outside).expect("outside dir");
        fs::rename(root.join("assets"), &outside).expect("move assets out");
        std::os::unix::fs::symlink(&outside, root.join("assets")).expect("link assets");
        let error = BundleFacts::load(&root)
            .err()
            .expect("a symlinked assets root is refused");
        assert!(
            error.contains("traverses a symbolic link"),
            "unexpected refusal: {error}"
        );
    }

    #[test]
    fn a_symlinked_component_source_ancestor_is_refused() {
        let guard = tempfile::tempdir().expect("bundle tempdir");
        let root = guard.path().join("bundle");
        let model = serde_json::json!({
            "entry": "assets/model.xml",
            "resources": ["assets/model.xml"],
        });
        write_bundle(&root, Some(model));
        // Stage one component source under a symlinked ancestor.
        let outside = guard.path().join("outside");
        let source = outside.join("components/d1");
        fs::create_dir_all(&source).expect("outside source");
        fs::write(source.join("model.xml"), "<model/>").expect("component model");
        let ancestor = root.join("assets/components");
        fs::create_dir_all(&ancestor).expect("ancestor dir");
        std::os::unix::fs::symlink(&outside, root.join("assets/linked")).expect("link ancestor");
        let manifest_path = root.join("manifest.json");
        let mut manifest: serde_json::Value =
            serde_json::from_slice(&fs::read(&manifest_path).expect("manifest"))
                .expect("decode manifest");
        manifest["components"] = serde_json::json!([{
            "instance": "d1",
            "driver": true,
            "package": "fixture-d1",
            "source": "local",
            "mount_site": "mount",
            "definition": {
                "schema": "phoxal/component/v0",
                "model": {"file": "model.xml", "root_body": "root"},
                "capabilities": {},
                "assets": [],
            },
        }]);
        manifest["component_sources"] = serde_json::json!({"d1": "assets/linked/components/d1"});
        manifest["instances"]
            .as_array_mut()
            .expect("instances")
            .push(serde_json::json!({"id": "d1", "role": "driver", "artifact": ID}));
        fs::write(
            &manifest_path,
            serde_json::to_vec(&manifest).expect("encode"),
        )
        .expect("rewrite manifest");

        let facts = BundleFacts::load(&root).expect("the manifest itself still loads");
        let component = facts.components.get("d1").cloned().expect("component");
        let error = facts
            .component_closed_model(&component, Path::new("model.xml"))
            .expect_err("a symlinked component source ancestor is refused");
        assert!(
            error.contains("traverses a symbolic link"),
            "unexpected refusal: {error}"
        );
    }
}
