use crate::mujoco::ClosedModel;
use crate::mujoco::Resource;
use phoxal_artifact_format::bundle::BundleComponent;
use phoxal_artifact_format::bundle::BundleManifest;
use phoxal_artifact_format::bundle::BundleProvenance;
use phoxal_artifact_format::bundle::BundleSimulation;
use phoxal_artifact_format::bundle::digest_source_files;
use serde::Deserialize;
use sha2::Digest;
use sha2::Sha256;
use std::collections::BTreeSet;
use std::fs;
use std::path::Component;
use std::path::Path;
use std::path::PathBuf;

pub(super) const BUNDLE_SCHEMA: &str = "phoxal/bundle/v0";

pub(super) const PROVENANCE_SCHEMA: &str = BUNDLE_SCHEMA;

pub(super) const MODEL_ASSET_PREFIX: &str = "assets/";

pub(super) const SOURCE_PREFIX: &str = "source/";

pub(super) struct BundleFacts {
    pub(super) root: PathBuf,
    pub(super) manifest: BundleManifest,
    pub(super) provenance: BundleProvenance,
    #[cfg(feature = "rendering")]
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
        if manifest.schema != BUNDLE_SCHEMA {
            return Err(format!(
                "bundle manifest schema is {}, expected {BUNDLE_SCHEMA}",
                manifest.schema
            ));
        }
        let provenance =
            read_json::<BundleProvenance>(&root.join("provenance.json"), "bundle provenance")?;
        if provenance.schema != PROVENANCE_SCHEMA {
            return Err(format!(
                "bundle provenance schema is {}, expected {PROVENANCE_SCHEMA}",
                provenance.schema
            ));
        }
        if provenance.source_tree.path != "source" {
            return Err(format!(
                "bundle source tree path is {}, expected source",
                provenance.source_tree.path
            ));
        }
        #[cfg(feature = "rendering")]
        let simulation = manifest.simulation.clone();
        let facts = Self {
            root,
            manifest,
            provenance,
            #[cfg(feature = "rendering")]
            simulation,
        };
        facts.validate_source_tree()?;
        facts.validate_model_closure()?;
        Ok(facts)
    }

    pub(super) fn validate_source_tree(&self) -> Result<(), String> {
        let source_root = self.root.join(SOURCE_PREFIX);
        let source_metadata = fs::symlink_metadata(&source_root).map_err(|error| {
            format!(
                "cannot inspect bundle source tree {}: {error}",
                source_root.display()
            )
        })?;
        if source_metadata.file_type().is_symlink() || !source_metadata.is_dir() {
            return Err("bundle source tree is not a regular non-symlink directory".to_owned());
        }
        let mut paths = BTreeSet::new();
        for file in &self.provenance.source_tree.files {
            validate_relative_path(&file.path, "source file")?;
            if !paths.insert(file.path.as_str()) {
                return Err(format!("bundle source file {:?} is duplicated", file.path));
            }
            let path = regular_file_under(&source_root, Path::new(&file.path), &file.path)?;
            verify_digest(&path, file.bytes, &file.sha256)?;
        }
        let digest = digest_source_files(&self.provenance.source_tree.files);
        if digest != self.provenance.source_tree.digest {
            return Err(format!(
                "bundle source closure digest {} does not match staged files {}",
                self.provenance.source_tree.digest, digest
            ));
        }
        Ok(())
    }

    pub(super) fn validate_model_closure(&self) -> Result<(), String> {
        let model = self
            .provenance
            .model
            .as_ref()
            .ok_or_else(|| "bundle provenance has no authored model".to_owned())?;
        let closure =
            self.provenance.model_closure.as_ref().ok_or_else(|| {
                "bundle provenance has no closed model/resource closure".to_owned()
            })?;
        if closure.resources.is_empty() {
            return Err("bundle model closure has no resources".to_owned());
        }
        let mut names = BTreeSet::new();
        for resource in &closure.resources {
            let relative = resource
                .path
                .strip_prefix(MODEL_ASSET_PREFIX)
                .ok_or_else(|| format!("model resource {} is outside assets/", resource.path))?;
            validate_relative_path(relative, "model resource")?;
            if !names.insert(relative.to_owned()) {
                return Err(format!("bundle model resource {relative:?} is duplicated"));
            }
            let path = regular_file_under(
                &self.root.join("assets"),
                Path::new(relative),
                &resource.path,
            )?;
            verify_digest(&path, resource.bytes, &resource.sha256)?;
        }
        let entry = closure
            .entry
            .strip_prefix(MODEL_ASSET_PREFIX)
            .ok_or_else(|| format!("model entry {} is outside assets/", closure.entry))?;
        validate_relative_path(entry, "model entry")?;
        if !names.contains(entry) {
            return Err(format!("bundle model entry {entry:?} is not staged"));
        }
        let entry_resource = closure
            .resources
            .iter()
            .find(|resource| resource.path == closure.entry)
            .ok_or_else(|| format!("bundle model closure is missing entry {}", closure.entry))?;
        if entry_resource.sha256 != model.sha256 || entry_resource.bytes != model.bytes {
            return Err(format!(
                "bundle model source {} differs from staged closure entry {}",
                model.path, closure.entry
            ));
        }
        let resources = closure
            .resources
            .iter()
            .map(|resource| {
                let name = resource
                    .path
                    .strip_prefix(MODEL_ASSET_PREFIX)
                    .expect("model resource was validated above")
                    .to_owned();
                let bytes = fs::read(self.root.join(&resource.path)).map_err(|error| {
                    format!("cannot read model resource {}: {error}", resource.path)
                })?;
                Resource::new(name, bytes).map_err(|error| error.to_string())
            })
            .collect::<Result<Vec<_>, _>>()?;
        let closed = ClosedModel::new(entry, resources).map_err(|error| error.to_string())?;
        if closed.digest_hex() != closure.digest {
            return Err(format!(
                "bundle model closure digest {} does not match staged resources {}",
                closure.digest,
                closed.digest_hex()
            ));
        }
        Ok(())
    }

    pub(super) fn root_closed_model(&self) -> Result<ClosedModel, String> {
        let closure =
            self.provenance.model_closure.as_ref().ok_or_else(|| {
                "bundle provenance has no closed model/resource closure".to_owned()
            })?;
        let entry = closure
            .entry
            .strip_prefix(MODEL_ASSET_PREFIX)
            .ok_or_else(|| format!("model entry {} is outside assets/", closure.entry))?;
        let resources = closure
            .resources
            .iter()
            .map(|resource| {
                let name = resource
                    .path
                    .strip_prefix(MODEL_ASSET_PREFIX)
                    .ok_or_else(|| format!("model resource {} is outside assets/", resource.path))?
                    .to_owned();
                let bytes = fs::read(self.root.join(&resource.path)).map_err(|error| {
                    format!("cannot read model resource {}: {error}", resource.path)
                })?;
                Resource::new(name, bytes).map_err(|error| error.to_string())
            })
            .collect::<Result<Vec<_>, _>>()?;
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
        let prefix = self.component_source_prefix(component, entry)?;
        let resources = self
            .provenance
            .source_tree
            .files
            .iter()
            .filter_map(|file| {
                let relative = file.path.strip_prefix(&prefix)?;
                let relative = relative.strip_prefix('/')?;
                Some((relative.to_owned(), file))
            })
            .filter(|(relative, _)| !relative.is_empty())
            .map(|(relative, file)| {
                validate_relative_path(&relative, "component resource")?;
                let path = regular_file_under(
                    &self.root.join(SOURCE_PREFIX),
                    Path::new(&file.path),
                    &file.path,
                )?;
                verify_digest(&path, file.bytes, &file.sha256)?;
                let bytes = fs::read(&path).map_err(|error| {
                    format!("cannot read component resource {}: {error}", file.path)
                })?;
                Resource::new(relative, bytes).map_err(|error| error.to_string())
            })
            .collect::<Result<Vec<_>, _>>()?;
        if !resources.iter().any(|resource| resource.name() == entry) {
            return Err(format!(
                "component {} source closure has no {}",
                component.instance, entry
            ));
        }
        ClosedModel::new(entry, resources).map_err(|error| error.to_string())
    }

    pub(super) fn component_source_prefix(
        &self,
        component: &BundleComponent,
        entry: &str,
    ) -> Result<String, String> {
        let source = self
            .provenance
            .sources
            .iter()
            .find(|source| {
                source.package_id == component.package_id && source.source == component.source
            })
            .ok_or_else(|| {
                format!(
                    "component {} has no exact source provenance for package {}",
                    component.instance, component.package_id
                )
            })?;
        let source_entry = source
            .files
            .iter()
            .find(|file| file.path == entry)
            .ok_or_else(|| {
                format!(
                    "component {} source record has no model entry {}",
                    component.instance, entry
                )
            })?;
        let mut prefixes = BTreeSet::new();
        for staged_entry in &self.provenance.source_tree.files {
            if staged_entry.sha256 != source_entry.sha256
                || staged_entry.bytes != source_entry.bytes
            {
                continue;
            }
            let Some(prefix) = staged_entry
                .path
                .strip_suffix(&format!("/{entry}"))
                .or_else(|| (staged_entry.path == entry).then_some(""))
            else {
                continue;
            };
            let matches_source = source.files.iter().all(|file| {
                let path = if prefix.is_empty() {
                    file.path.clone()
                } else {
                    format!("{prefix}/{}", file.path)
                };
                self.provenance.source_tree.files.iter().any(|staged| {
                    staged.path == path
                        && staged.sha256 == file.sha256
                        && staged.bytes == file.bytes
                })
            });
            if matches_source {
                prefixes.insert(prefix.to_owned());
            }
        }
        match prefixes.len() {
            1 => Ok(prefixes.into_iter().next().expect("one prefix")),
            0 => Err(format!(
                "component {} source closure is not present under its exact package identity",
                component.instance
            )),
            _ => Err(format!(
                "component {} source closure has ambiguous staged package locations",
                component.instance
            )),
        }
    }
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

pub(super) fn verify_digest(
    path: &Path,
    expected_bytes: u64,
    expected_sha256: &str,
) -> Result<(), String> {
    let bytes =
        fs::read(path).map_err(|error| format!("cannot read {}: {error}", path.display()))?;
    let actual_bytes = bytes.len() as u64;
    let actual_sha256 = format!("{:x}", Sha256::digest(&bytes));
    if actual_bytes != expected_bytes || actual_sha256 != expected_sha256 {
        return Err(format!(
            "bundle file {} has {actual_bytes} bytes and SHA-256 {actual_sha256}, expected {expected_bytes} bytes and {expected_sha256}",
            path.display()
        ));
    }
    Ok(())
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
