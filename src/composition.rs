use crate::bundle::BundleFacts;
use crate::bundle::regular_file;
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
        bundle.manifest.robot_id
    )
}

pub(super) fn load_composed_model(scene: &Path, bundle: &BundleFacts) -> Result<Model, String> {
    let scene = scene
        .canonicalize()
        .map_err(|error| format!("cannot canonicalize scene {}: {error}", scene.display()))?;
    regular_file(&scene, &scene.display().to_string())?;
    let scene = ClosedModel::from_file(&scene)
        .map_err(|error| format!("cannot close scene {}: {error}", scene.display()))?;
    let robot_model = bundle
        .provenance
        .model
        .as_ref()
        .ok_or_else(|| "bundle provenance has no authored robot model".to_owned())?;
    let declared_robot_model = bundle
        .manifest
        .document
        .robot
        .model
        .as_ref()
        .ok_or_else(|| "bundle manifest has no authored robot model".to_owned())?;
    let declared_robot_model = declared_robot_model.to_string_lossy().replace('\\', "/");
    if declared_robot_model != robot_model.path {
        return Err(format!(
            "bundle robot model {} disagrees with manifest model {}",
            robot_model.path, declared_robot_model
        ));
    }
    if bundle.manifest.robot_id != bundle.manifest.document.robot.id {
        return Err(format!(
            "bundle robot identity {} disagrees with authored robot id {}",
            bundle.manifest.robot_id, bundle.manifest.document.robot.id
        ));
    }
    let robot = bundle.root_closed_model()?;
    let robot_root = unique_direct_root_body(&robot).map_err(|error| error.to_string())?;
    let mut attachments = Vec::new();
    for (instance, selection) in &bundle.manifest.document.robot.components {
        let selected = bundle
            .manifest
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
        if selected.dependency_key != selection.component {
            return Err(format!(
                "component {instance} dependency key {} disagrees with robot selection {}",
                selected.dependency_key, selection.component
            ));
        }
        let component = bundle.component_closed_model(selected, &selected.definition.model.file)?;
        attachments.push(
            ComponentAttachment::new(
                instance.clone(),
                component,
                selected.mount_site.clone(),
                selected.definition.model.root_body.clone(),
            )
            .map_err(|error| error.to_string())?,
        );
    }
    let composition = SceneComposition::new(
        scene,
        robot,
        bundle.manifest.document.robot.id.clone(),
        "robot_mount",
        robot_root,
        attachments,
    )
    .map_err(|error| error.to_string())?;
    #[cfg(any(test, feature = "rendering"))]
    let model = composition
        .compile()
        .map_err(|error| error.to_string())?;
    #[cfg(not(any(test, feature = "rendering")))]
    let model = {
        // Native probe path cannot compile the composition because
        // --features native omits the compilation code paths. The
        // composition was successfully built (the SceneComposition was
        // constructed above) so the bundle can still be probed for
        // identity, providers, and actuation bindings.
        let _ = composition;
        return Err(
            "phoxal-simulator-mujoco --features native cannot compile a \
             composed model; only --probe is supported in this configuration"
                .to_owned(),
        );
    };
    Ok(model)
}
