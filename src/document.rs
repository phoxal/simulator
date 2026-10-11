//! Authored scene/robot sources for the live session, read-only.
//! Simulators consume complete bundles and scenes without modifying them,
//! and sessions are never persisted as authored documents.
use crate::mujoco::{ClosedModel, Resource};
use std::path::{Path, PathBuf};

pub(crate) const DEFAULT_SCENE: &[u8] = include_bytes!("../resources/default-scene.xml");

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct SceneSource {
    pub path: Option<PathBuf>,
    entry: String,
    resources: Vec<SavedResource>,
    identity: String,
}
#[derive(Clone, Debug, PartialEq)]
struct SavedResource {
    name: String,
    bytes: Vec<u8>,
}
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct RobotSource {
    pub build: PathBuf,
    pub manifest: String,
    pub model: String,
    pub target: String,
    pub position: [f64; 3],
    pub quaternion: [f64; 4],
}
#[derive(Clone, Debug)]
pub(crate) struct Document {
    pub definition: Definition,
}
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Definition {
    pub scene: SceneSource,
    pub robot: Option<RobotSource>,
}
impl SceneSource {
    pub fn from_closed(path: Option<PathBuf>, closed: &ClosedModel) -> Self {
        Self {
            path,
            entry: closed.entry().into(),
            resources: closed
                .resources()
                .map(|r| SavedResource {
                    name: r.name().into(),
                    bytes: r.bytes().into(),
                })
                .collect(),
            identity: crate::mujoco::ModelIdentity(closed.digest()).to_hex(),
        }
    }
    pub fn close(&self) -> Result<ClosedModel, String> {
        let limits = crate::mujoco::ResourceLimits::default();
        if self.resources.len() > limits.max_resources
            || self
                .resources
                .iter()
                .any(|r| r.bytes.len() > limits.max_resource_bytes)
            || self
                .resources
                .iter()
                .map(|r| r.bytes.len() as u64)
                .sum::<u64>()
                > limits.max_closure_bytes as u64
        {
            return Err("Scene closure exceeds admission limits".into());
        }
        let closed = ClosedModel::new(
            &self.entry,
            self.resources
                .iter()
                .map(|r| Resource::new(&r.name, r.bytes.clone()).map_err(|e| e.to_string()))
                .collect::<Result<Vec<_>, _>>()
                .map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        if crate::mujoco::ModelIdentity(closed.digest()).to_hex() != self.identity {
            return Err("Scene identity does not match its exact resource bytes".into());
        }
        Ok(closed)
    }
    pub fn verify_live(&self) -> Result<(), String> {
        let expected = self.close()?;
        if let Some(path) = &self.path {
            crate::bundle::regular_file(path, "selected scene")?;
            let actual = ClosedModel::from_referenced_file(path).map_err(|e| e.to_string())?;
            if actual != expected {
                return Err(format!(
                    "Scene resources changed: {}. Select the scene explicitly to load the changed content.",
                    path.display()
                ));
            }
        }
        Ok(())
    }
}
impl Document {
    pub fn default_scene() -> Result<Self, String> {
        let scene = ClosedModel::from_xml(DEFAULT_SCENE).map_err(|e| e.to_string())?;
        Ok(Self {
            definition: Definition {
                scene: SceneSource::from_closed(None, &scene),
                robot: None,
            },
        })
    }
    pub fn scene(path: &Path) -> Result<Self, String> {
        crate::bundle::regular_file(path, "selected scene")?;
        let path = path.canonicalize().map_err(|e| e.to_string())?;
        let closed = ClosedModel::from_referenced_file(&path).map_err(|e| e.to_string())?;
        Ok(Self {
            definition: Definition {
                scene: SceneSource::from_closed(Some(path), &closed),
                robot: None,
            },
        })
    }
}

/// A session departure that must wait for the live world to stop and
/// cleanup before it may proceed. There is no unsaved-document decision:
/// sessions never modify authored inputs.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum PendingAction {
    New,
    OpenScene(PathBuf),
    Close,
}
#[derive(Default)]
pub(crate) struct Departure {
    pub pending: Option<PendingAction>,
}
impl Departure {
    pub fn begin(&mut self, action: PendingAction) {
        self.pending = Some(action);
    }
    pub fn take_ready(&mut self, stopped: bool, cleanup_failed: bool) -> Option<PendingAction> {
        if stopped
            && (!cleanup_failed || matches!(self.pending, Some(PendingAction::Close)))
            && self.pending.is_some()
        {
            self.pending.take()
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn departure_waits_for_confirmed_cleanup_and_close_survives_uncertainty() {
        let mut d = Departure::default();
        d.begin(PendingAction::New);
        assert!(d.take_ready(false, false).is_none());
        assert!(d.take_ready(true, true).is_none());
        assert_eq!(d.take_ready(true, false), Some(PendingAction::New));
        d.begin(PendingAction::Close);
        assert_eq!(d.take_ready(true, true), Some(PendingAction::Close));
    }
    #[test]
    fn session_documents_carry_no_persistence_or_dirty_state() {
        let document = Document::default_scene().unwrap();
        assert_eq!(
            document
                .definition
                .scene
                .close()
                .unwrap()
                .resources()
                .next()
                .unwrap()
                .bytes(),
            DEFAULT_SCENE
        );
        let scene = tempfile::tempdir().unwrap();
        let path = scene.path().join("scene.xml");
        std::fs::write(&path, DEFAULT_SCENE).unwrap();
        let selected = Document::scene(&path).unwrap();
        std::fs::write(&path, b"<mujoco/>").unwrap();
        assert!(
            selected
                .definition
                .scene
                .verify_live()
                .unwrap_err()
                .contains("changed")
        );
    }
}
