use crate::bundle::regular_file;
use crate::bundle::{BundleFacts, component_definition};
use crate::mujoco::ClosedModel;
use crate::mujoco::ComponentAttachment;
use crate::mujoco::Model;
use crate::mujoco::SceneComposition;
use crate::mujoco::unique_direct_root_body;
use phoxal::artifact::document::Source;
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
    let scene = ClosedModel::from_file(&scene)
        .map_err(|error| format!("cannot close scene {}: {error}", scene.display()))?;
    bundle
        .model
        .as_ref()
        .ok_or_else(|| "bundle has no robot model assets".to_owned())?;
    bundle
        .robot
        .model
        .as_ref()
        .ok_or_else(|| "bundle manifest has no authored robot model".to_owned())?;
    if bundle.robot_id != bundle.robot.id {
        return Err(format!(
            "bundle robot identity {} disagrees with authored robot id {}",
            bundle.robot_id, bundle.robot.id
        ));
    }
    let robot = bundle.root_closed_model()?;
    let robot_root = unique_direct_root_body(&robot).map_err(|error| error.to_string())?;
    let mut attachments = Vec::new();
    for (instance, selection) in &bundle.robot.components {
        let selected = bundle
            .components
            .iter()
            .find(|component| component.instance == *instance)
            .ok_or_else(|| format!("component {instance} has no resolved bundle record"))?;
        if selected.mount_site != selection.mount_site {
            return Err(format!(
                "component {instance} mount_site {} disagrees with resolved bundle record {}",
                selection.mount_site, selected.mount_site
            ));
        }
        let selected_name = match &selection.source {
            Source::Path(_) => None,
            Source::Package(package) => Some(package.name.as_str()),
            Source::Git(git) => Some(git.name.as_str()),
        };
        if let Some(name) = selected_name
            && selected.package != name
        {
            return Err(format!(
                "component {instance} package {} disagrees with robot selection {name}",
                selected.package
            ));
        }
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
        bundle.robot.id.clone(),
        "robot_mount",
        robot_root,
        attachments,
    )
    .map_err(|error| error.to_string())?
    .compile()
    .map_err(|error| error.to_string())
}
