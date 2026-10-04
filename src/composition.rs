use crate::bundle::regular_file;
use crate::bundle::{BundleFacts, component_definition};
use crate::mujoco::ClosedModel;
use crate::mujoco::ComponentAttachment;
use crate::mujoco::Model;
use crate::mujoco::SceneComposition;
use crate::mujoco::unique_direct_root_body;
use std::path::Path;

pub(super) const COMPONENT_NAMESPACE_SEPARATOR: &str = "__";

pub(super) fn native_component_prefix(bundle: &BundleFacts, instance: &str) -> String {
    format!(
        "{}{COMPONENT_NAMESPACE_SEPARATOR}{instance}{COMPONENT_NAMESPACE_SEPARATOR}",
        bundle.robot_id
    )
}

pub(super) fn load_composed_model(scene: &Path, bundle: &BundleFacts) -> Result<Model, String> {
    let scene = scene
        .canonicalize()
        .map_err(|error| format!("cannot canonicalize scene {}: {error}", scene.display()))?;
    regular_file(&scene, &scene.display().to_string())?;
    let scene = ClosedModel::from_referenced_file(&scene)
        .map_err(|error| format!("cannot close scene {}: {error}", scene.display()))?;
    bundle
        .model
        .as_ref()
        .ok_or_else(|| "bundle has no robot model assets".to_owned())?;
    let robot = bundle.root_closed_model()?;
    let robot_root = unique_direct_root_body(&robot).map_err(|error| error.to_string())?;
    let mut attachments = Vec::new();
    for (instance, selected) in &bundle.components {
        let (component_model, _) = component_definition(&selected.definition);
        let component = bundle.component_closed_model(selected, &component_model.file)?;
        attachments.push(
            ComponentAttachment::new(
                instance.clone(),
                component,
                selected.mount_site.clone(),
                component_model.root_body.clone(),
            )
            .map_err(|error| error.to_string())?,
        );
    }
    SceneComposition::new(
        scene,
        robot,
        bundle.robot_id.clone(),
        "robot_mount",
        robot_root,
        attachments,
    )
    .map_err(|error| error.to_string())?
    .compile()
    .map_err(|error| error.to_string())
}
