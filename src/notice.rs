//! Private presentation of one primary failure, its recovery action and complete details.
#[derive(Clone, Debug)]
pub(crate) struct Notice {
    pub(crate) primary: String,
    pub(crate) action: String,
    pub(crate) details: String,
}

impl Notice {
    pub(crate) fn new(
        primary: impl Into<String>,
        action: impl Into<String>,
        details: impl Into<String>,
    ) -> Self {
        let primary = primary.into();
        let action = action.into();
        let details = details.into();
        let mut rendered = primary.clone();
        if !action.is_empty() {
            rendered.push_str(&format!("\nNext: {action}"));
        }
        if !details.is_empty() {
            rendered.push('\n');
            rendered.push_str(&details);
        }
        Self {
            primary,
            action,
            details: rendered,
        }
    }
}

impl From<String> for Notice {
    fn from(details: String) -> Self {
        let primary = details
            .lines()
            .next()
            .unwrap_or("Simulation failed")
            .to_owned();
        // Unclassified execution errors have no inferred action from untrusted text.
        let action =
            "inspect the execution error details and correct the reported cause before retrying."
                .to_owned();
        Self {
            primary,
            action: action.clone(),
            details: format!("{details}\nNext: {action}"),
        }
    }
}

impl From<&str> for Notice {
    fn from(details: &str) -> Self {
        details.to_owned().into()
    }
}

impl std::fmt::Display for Notice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.details)
    }
}
