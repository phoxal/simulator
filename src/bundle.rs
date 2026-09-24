use crate::mujoco::ClosedModel;
use crate::mujoco::Resource;
use phoxal::artifact::bundle::{
    BundleComponent, BundleExecutable, BundleManifest, BundleModelAssets, BundleSimulation,
};
use phoxal::artifact::document::{
    CapabilityDeclaration, ComponentDocument, ComponentModel, RobotDocument, RobotSection,
};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Component;
use std::path::Path;
use std::path::PathBuf;

pub(super) const MODEL_ASSET_PREFIX: &str = "assets/";

pub(super) struct BundleFacts {
    pub(super) root: PathBuf,
    pub(super) robot_id: String,
    pub(super) robot: RobotSection,
    pub(super) executables: Vec<BundleExecutable>,
    pub(super) components: Vec<BundleComponent>,
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
        let manifest = read_json::<BundleManifest>(&root.join("manifest.json"), "bundle manifest")?;
        let BundleManifest::V0 {
            robot_id,
            executables,
            components,
            component_sources,
            model,
            simulation,
            ..
        } = manifest;
        let document_path = root.join("robot.yaml");
        regular_file(&document_path, "robot.yaml")?;
        let document: RobotDocument = serde_yaml::from_slice(
            &fs::read(&document_path)
                .map_err(|error| format!("cannot read robot.yaml: {error}"))?,
        )
        .map_err(|error| format!("cannot parse robot.yaml: {error}"))?;
        let RobotDocument::V0 { robot, .. } = document;
        let facts = Self {
            root,
            robot_id,
            robot,
            executables,
            components,
            component_sources,
            model,
            simulation,
        };
        if facts.model.is_some() {
            facts.root_closed_model()?;
        }
        Ok(facts)
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
        let source_root = self.root.join(relative);
        let metadata = fs::symlink_metadata(&source_root).map_err(|error| {
            format!(
                "cannot inspect component source {}: {error}",
                source_root.display()
            )
        })?;
        if !metadata.is_dir() || metadata.file_type().is_symlink() {
            return Err(format!(
                "component source {} is not a regular directory",
                source_root.display()
            ));
        }
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

pub(super) fn read_json<T: for<'de> Deserialize<'de>>(
    path: &Path,
    label: &str,
) -> Result<T, String> {
    let bytes = fs::read(path)
        .map_err(|error| format!("cannot read {label} {}: {error}", path.display()))?;
    serde_json::from_slice(&bytes)
        .map_err(|error| format!("cannot parse {label} {}: {error}", path.display()))
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
