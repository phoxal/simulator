use super::binding::BoundActuation;
use super::binding::NativeActuationBindingFact;
use super::binding::NativeObservationBindingFact;
use super::binding::bind_actuation;
use super::binding::bind_observation;
use super::config::ActuationDeclaration;
use super::config::NativeControlMode;
use super::config::ObservationBinding;
use super::observations::BoundObservation;
use crate::cadence::Cadence;
use crate::remote::ActuationBinding;
use crate::remote::NativeProvider;
use crate::remote::NativeProviderError;
use crate::remote::ProviderSet;
use crate::remote::validate_bindings_for_model;
use phoxal::communication::simulation::Actuation;
use phoxal::communication::simulation::Observation;
use phoxal_mujoco::Model;
use phoxal_mujoco::StateSnapshot;
use phoxal_mujoco::Workspace;
use prost::Message;
use prost::Name;
use std::collections::BTreeMap;
use std::collections::BTreeSet;

/// A production-shaped fixed reference provider for the currently maintained
/// MuJoCo component contracts.
///
/// The provider is immutable after construction apart from reset-local state
/// owned by the trait interface.  It emits only native sensor/site values and
/// accepts only explicitly mapped motion targets.
#[derive(Debug)]
pub struct ComponentProvider {
    pub(super) providers: ProviderSet,
    pub(super) observations: Vec<(BoundObservation, Vec<(String, String)>)>,
    cadence: BTreeMap<(String, String), Cadence>,
    pub(super) observation_facts: Vec<NativeObservationBindingFact>,
    pub(super) actuations: Vec<BoundActuation>,
    pub(super) actuation_facts: Vec<NativeActuationBindingFact>,
    pub(super) actuation_bindings: Vec<ActuationBinding>,
    pub(super) render_workspace: Option<Workspace>,
}

impl ComponentProvider {
    /// Builds and validates a fixed provider against one compiled model.
    pub fn new(
        model: &Model,
        providers: ProviderSet,
        observations: impl IntoIterator<Item = ObservationBinding>,
        actuations: impl IntoIterator<Item = ActuationDeclaration>,
        cadence: BTreeMap<(String, String), Cadence>,
    ) -> Result<Self, NativeProviderError> {
        let mut bound_observations = Vec::new();
        let mut observation_facts = Vec::new();
        let mut routes = BTreeSet::new();
        for config in observations {
            let (bound, facts) = bind_observation(model, &providers, config)?;
            let mut source_routes = Vec::new();
            for fact in facts {
                let key = (fact.service_instance.clone(), fact.port.clone());
                if !routes.insert(key.clone()) {
                    return Err(NativeProviderError::InvalidPayload(format!(
                        "reference provider route {}/{} is configured more than once",
                        key.0, key.1
                    )));
                }
                source_routes.push(key);
                observation_facts.push(fact);
            }
            bound_observations.push((bound, source_routes));
        }
        let expected_routes = providers
            .requirements()
            .iter()
            .map(|requirement| {
                (
                    requirement.service_instance.clone(),
                    requirement.port.clone(),
                )
            })
            .collect::<BTreeSet<_>>();
        if cadence.keys().cloned().collect::<BTreeSet<_>>() != expected_routes {
            return Err(NativeProviderError::InvalidPayload(
                "source cadence must cover every provider route exactly".into(),
            ));
        }
        if expected_routes != routes {
            let missing = expected_routes
                .difference(&routes)
                .map(|(service, port)| format!("{service}/{port}"))
                .collect::<Vec<_>>();
            let extra = routes
                .difference(&expected_routes)
                .map(|(service, port)| format!("{service}/{port}"))
                .collect::<Vec<_>>();
            return Err(NativeProviderError::InvalidPayload(format!(
                "reference provider requirements and bindings differ; missing {:?}, extra {:?}",
                missing, extra
            )));
        }

        let mut bound_actuations = Vec::new();
        let mut actuation_facts = Vec::new();
        let mut actuation_bindings = Vec::new();
        let mut actuation_routes = BTreeSet::new();
        for config in actuations {
            let (bound, fact) = bind_actuation(model, &config)?;
            let route = (
                bound.binding.service_instance().to_owned(),
                bound.binding.port().to_owned(),
            );
            if !actuation_routes.insert(route.clone()) {
                return Err(NativeProviderError::InvalidActuation(format!(
                    "reference actuation route {}/{} is configured more than once",
                    route.0, route.1
                )));
            }
            actuation_bindings.push(bound.binding.clone());
            actuation_facts.push(fact);
            bound_actuations.push(bound);
        }
        validate_bindings_for_model(model, &actuation_bindings)
            .map_err(|error| NativeProviderError::InvalidActuation(error.to_string()))?;

        bound_actuations.sort_by(|left, right| {
            left.binding
                .service_instance()
                .cmp(right.binding.service_instance())
                .then_with(|| left.binding.port().cmp(right.binding.port()))
        });
        actuation_bindings.sort_by(|left, right| {
            left.service_instance()
                .cmp(right.service_instance())
                .then_with(|| left.port().cmp(right.port()))
        });

        observation_facts.sort_by(|left, right| {
            left.service_instance
                .cmp(&right.service_instance)
                .then_with(|| left.port.cmp(&right.port))
        });
        actuation_facts.sort_by(|left, right| {
            left.service_instance
                .cmp(&right.service_instance)
                .then_with(|| left.port.cmp(&right.port))
        });

        Ok(Self {
            providers,
            cadence,
            observations: bound_observations,
            observation_facts,
            actuations: bound_actuations,
            actuation_facts,
            actuation_bindings,
            render_workspace: None,
        })
    }

    pub fn binding_evidence(&self) -> serde_json::Value {
        serde_json::json!({
            "observations": self.observation_facts.iter().map(|fact| serde_json::json!({
                "producer": fact.service_instance, "port": fact.port, "payload_fqn": fact.payload_fqn, "native_names": fact.native_names
            })).collect::<Vec<_>>(),
            "actuation": self.actuation_facts.iter().map(|fact| serde_json::json!({
                "producer": fact.service_instance, "port": fact.port, "payload_fqn": fact.payload_fqn,
                "actuators": fact.actuators.iter().map(|actuator| serde_json::json!({
                    "actuator_id": actuator.actuator_id, "native_name": actuator.native_name,
                    "control_mode": format!("{:?}", actuator.mode), "native_mode": format!("{:?}", actuator.native_mode)
                })).collect::<Vec<_>>()
            })).collect::<Vec<_>>()
        })
    }
}

impl NativeProvider for ComponentProvider {
    type Error = NativeProviderError;

    fn providers(&self) -> &ProviderSet {
        &self.providers
    }

    fn actuation_bindings(&self) -> &[ActuationBinding] {
        &self.actuation_bindings
    }

    fn observations(
        &mut self,
        model: &Model,
        state: &StateSnapshot,
        quantum_ns: u64,
    ) -> Result<Vec<Observation>, Self::Error> {
        if quantum_ns == 0 {
            return Err(NativeProviderError::InvalidPayload(
                "provider quantum must be positive".to_owned(),
            ));
        }
        if state.model_identity() != model.identity() {
            return Err(NativeProviderError::InvalidPayload(
                "provider state belongs to a different native model".to_owned(),
            ));
        }
        if self.observations.iter().any(|(binding, routes)| {
            binding.requires_workspace()
                && routes
                    .iter()
                    .any(|route| self.cadence[route].due(state.boundary()))
        }) {
            let workspace = match self.render_workspace.as_mut() {
                Some(workspace) if workspace.model().identity() == model.identity() => workspace,
                Some(_) => {
                    return Err(NativeProviderError::InvalidPayload(
                        "camera renderer workspace belongs to a different native model".to_owned(),
                    ));
                }
                None => {
                    self.render_workspace = Some(Workspace::new(model).map_err(|error| {
                        NativeProviderError::Unsupported(format!(
                            "native camera renderer could not initialize: {error}"
                        ))
                    })?);
                    self.render_workspace.as_mut().ok_or_else(|| {
                        NativeProviderError::Unsupported(
                            "native camera renderer initialization returned no workspace"
                                .to_owned(),
                        )
                    })?
                }
            };
            workspace
                .set_qpos(state.qpos())
                .map_err(|error| NativeProviderError::InvalidPayload(error.to_string()))?;
            workspace
                .set_qvel(state.qvel())
                .map_err(|error| NativeProviderError::InvalidPayload(error.to_string()))?;
            workspace
                .set_controls(state.controls())
                .map_err(|error| NativeProviderError::InvalidPayload(error.to_string()))?;
            workspace
                .forward()
                .map_err(|error| NativeProviderError::Unsupported(error.to_string()))?;
        }
        let mut observations = Vec::new();
        for (binding, routes) in &self.observations {
            let mut due = Vec::new();
            for route in routes {
                if self.cadence[route].due(state.boundary()) {
                    due.push(route);
                } else {
                    observations.push(crate::observations::not_due(
                        &route.0, &route.1, state, quantum_ns,
                    )?);
                }
            }
            if due.is_empty() {
                continue;
            }
            let encoded = binding.encode(
                &self.providers,
                model,
                state,
                quantum_ns,
                self.render_workspace.as_mut(),
            )?;
            observations.extend(encoded.into_iter().filter(|observation| {
                observation.membership.as_ref().is_some_and(|m| {
                    due.iter()
                        .any(|route| route.0 == m.producer && route.1 == m.port)
                })
            }));
        }
        observations.sort_by_key(|value| {
            value
                .membership
                .as_ref()
                .map(|m| (m.producer.clone(), m.port.clone()))
        });
        self.providers
            .validate_observations(&observations, state.boundary(), quantum_ns)
            .map_err(|error| NativeProviderError::InvalidPayload(error.to_string()))?;
        Ok(observations)
    }

    fn controls(
        &mut self,
        model: &Model,
        actuation: &[Actuation],
    ) -> Result<Vec<f64>, Self::Error> {
        let expected_fqn = phoxal_service_motion::ActuatorSetpoint::full_name();
        if actuation.len() != self.actuations.len() {
            return Err(NativeProviderError::InvalidActuation(format!(
                "actuation cut has {}, expected {} configured outputs",
                actuation.len(),
                self.actuations.len()
            )));
        }
        let mut routes = BTreeSet::new();
        let mut controls = vec![0.0; model.counts().controls];
        let mut covered = BTreeSet::new();
        for binding in &self.actuations {
            if binding
                .targets
                .iter()
                .any(|target| target.binding.native.model_identity() != model.identity())
            {
                return Err(NativeProviderError::InvalidActuation(
                    "reference actuator binding belongs to a different native model".to_owned(),
                ));
            }
            let route = (binding.binding.service_instance(), binding.binding.port());
            let item = actuation
                .iter()
                .find(|item| {
                    item.membership
                        .as_ref()
                        .is_some_and(|m| (m.producer.as_str(), m.port.as_str()) == route)
                })
                .ok_or_else(|| {
                    NativeProviderError::InvalidActuation(format!(
                        "missing actuation {}/{}",
                        route.0, route.1
                    ))
                })?;
            if !routes.insert(route) {
                return Err(NativeProviderError::InvalidActuation(format!(
                    "actuation route {}/{} is duplicated",
                    route.0, route.1
                )));
            }
            if item.payload.is_empty() {
                return Err(NativeProviderError::InvalidActuation(format!(
                    "actuation {}/{} has an empty payload",
                    route.0, route.1
                )));
            }
            if binding.binding.payload_fqn() != expected_fqn {
                return Err(NativeProviderError::InvalidActuation(format!(
                    "actuation {}/{} uses payload {}, expected {expected_fqn}",
                    route.0,
                    route.1,
                    binding.binding.payload_fqn()
                )));
            }
            let setpoint = phoxal_service_motion::ActuatorSetpoint::decode(item.payload.as_slice())?;
            setpoint
                .validate_for(binding.targets.iter().map(|target| target.wire_id.as_str()))
                .map_err(|error| NativeProviderError::InvalidActuation(error.to_string()))?;
            for target in &binding.targets {
                let wire_target = setpoint
                    .targets
                    .iter()
                    .find(|candidate| candidate.actuator_id == target.wire_id)
                    .ok_or_else(|| {
                        NativeProviderError::InvalidActuation(format!(
                            "actuation {} omits configured target {}",
                            route.1, target.wire_id
                        ))
                    })?;
                let control = wire_target.control.as_ref().ok_or_else(|| {
                    NativeProviderError::InvalidActuation(format!(
                        "actuator {} has no control selection",
                        target.wire_id
                    ))
                })?;
                let value = match (target.mode, control) {
                    (
                        NativeControlMode::Torque,
                        phoxal_service_motion::actuator_target::Control::TorqueNm(value),
                    )
                    | (
                        NativeControlMode::Velocity,
                        phoxal_service_motion::actuator_target::Control::VelocityRadps(value),
                    ) => *value,
                    (NativeControlMode::Torque, _) => {
                        return Err(NativeProviderError::InvalidActuation(format!(
                            "actuator {} requires an explicit torque target",
                            target.wire_id
                        )));
                    }
                    (NativeControlMode::Velocity, _) => {
                        return Err(NativeProviderError::InvalidActuation(format!(
                            "actuator {} requires an explicit velocity target",
                            target.wire_id
                        )));
                    }
                };
                if !value.is_finite() {
                    return Err(NativeProviderError::InvalidActuation(format!(
                        "actuator {} control is not finite",
                        target.wire_id
                    )));
                }
                let control_index = target.binding.info.control_index;
                if let Some([lower, upper]) = model.control_range(control_index)?
                    && (value < lower || value > upper)
                {
                    return Err(NativeProviderError::InvalidActuation(format!(
                        "actuator {} control {value} is outside [{lower}, {upper}]",
                        target.wire_id
                    )));
                }
                if !covered.insert(control_index) {
                    return Err(NativeProviderError::InvalidActuation(format!(
                        "native control index {control_index} is mapped more than once"
                    )));
                }
                controls[control_index] = value;
            }
        }
        if routes.len() != actuation.len() || covered.len() != controls.len() {
            return Err(NativeProviderError::InvalidActuation(format!(
                "actuation covers {} routes and {} native controls, expected {} routes and {} controls",
                routes.len(),
                covered.len(),
                self.actuations.len(),
                controls.len()
            )));
        }
        Ok(controls)
    }

    fn reset(&mut self, _model: &Model, _state: &StateSnapshot) -> Result<(), Self::Error> {
        if let Some(workspace) = self.render_workspace.as_mut() {
            workspace
                .reset()
                .map_err(|error| NativeProviderError::Unsupported(error.to_string()))?;
        }
        Ok(())
    }
}
