//! Bounded archive extraction and contained resource copying.
use super::*;

pub(super) fn copy_bounded(
    input: &mut impl Read,
    output: &mut impl Write,
    limit: u64,
    cancel: &Cancellation,
) -> Result<u64, String> {
    let mut bytes = [0; 64 * 1024];
    let mut total = 0;
    loop {
        cancel.check()?;
        let count = input.read(&mut bytes).map_err(|e| e.to_string())?;
        if count == 0 {
            return Ok(total);
        }
        total += count as u64;
        if total > limit {
            return Err("Runtime expanded/download size limit exceeded".into());
        }
        output
            .write_all(&bytes[..count])
            .map_err(|e| e.to_string())?;
    }
}

pub(super) fn safe_relative(path: &Path) -> Result<(), String> {
    if path.as_os_str().is_empty()
        || path
            .components()
            .any(|part| !matches!(part, Component::Normal(_) | Component::CurDir))
    {
        return Err(format!("Unsafe runtime archive path: {}", path.display()));
    }
    Ok(())
}

pub(super) fn safe_link(parent: &Path, target: &Path) -> Result<(), String> {
    if target.is_absolute() {
        return Err("Absolute runtime symlink is not allowed".into());
    }
    let mut depth = parent.components().count();
    for part in target.components() {
        match part {
            Component::Normal(_) => depth += 1,
            Component::CurDir => (),
            Component::ParentDir if depth > 0 => depth -= 1,
            _ => return Err("Runtime symlink escapes its archive".into()),
        }
    }
    Ok(())
}

pub(super) fn extract_tar(
    archive: &Path,
    output: &Path,
    temporary: &Path,
    cancel: &Cancellation,
) -> Result<(), String> {
    let expanded = temporary.join("expanded");
    fs::create_dir(&expanded).map_err(|e| e.to_string())?;
    let file = File::open(archive).map_err(|e| e.to_string())?;
    let mut tar = tar::Archive::new(ExtractionReader {
        source: flate2::read::GzDecoder::new(file),
        cancel,
        remaining: MAX_EXPANDED,
    });
    let mut total = 0u64;
    for (index, entry) in tar.entries().map_err(|e| e.to_string())?.enumerate() {
        cancel.check()?;
        if index >= MAX_FILES {
            return Err("Runtime archive has too many entries".into());
        }
        let mut entry = entry.map_err(|e| e.to_string())?;
        let path = entry.path().map_err(|e| e.to_string())?.into_owned();
        safe_relative(&path)?;
        total = total
            .checked_add(entry.size())
            .ok_or("Runtime archive size overflow")?;
        if total > MAX_EXPANDED {
            return Err("Runtime archive expanded size exceeds limit".into());
        }
        let kind = entry.header().entry_type();
        if kind.is_symlink() {
            let target = entry
                .link_name()
                .map_err(|e| e.to_string())?
                .ok_or("Runtime symlink has no target")?;
            safe_link(path.parent().unwrap_or(Path::new("")), &target)?;
        } else if !kind.is_file() && !kind.is_dir() {
            return Err("Unsupported runtime archive entry type".into());
        }
        if !entry.unpack_in(&expanded).map_err(|e| e.to_string())? {
            return Err("Runtime archive escapes extraction directory".into());
        }
    }
    let source = expanded.join(format!("mujoco-{VERSION}"));
    let mut budget = CopyBudget::default();
    for name in ["lib", "include", "LICENSE", "THIRD_PARTY_NOTICES"] {
        copy_tree(
            &source,
            &source.join(name),
            &output.join(name),
            &mut budget,
            cancel,
        )?;
    }
    Ok(())
}

struct ExtractionReader<'a, R> {
    source: R,
    cancel: &'a Cancellation,
    remaining: u64,
}

impl<R: Read> Read for ExtractionReader<'_, R> {
    fn read(&mut self, bytes: &mut [u8]) -> std::io::Result<usize> {
        self.cancel.check().map_err(std::io::Error::other)?;
        let count = self.source.read(bytes)?;
        self.remaining = self
            .remaining
            .checked_sub(count as u64)
            .ok_or_else(|| std::io::Error::other("Runtime decompression exceeds size limit"))?;
        Ok(count)
    }
}

#[derive(Default)]
pub(super) struct CopyBudget {
    pub(super) bytes: u64,
    pub(super) files: usize,
}

pub(super) fn copy_tree(
    root: &Path,
    source: &Path,
    destination: &Path,
    budget: &mut CopyBudget,
    cancel: &Cancellation,
) -> Result<(), String> {
    cancel.check()?;
    budget.files += 1;
    if budget.files > MAX_FILES {
        return Err("Runtime contains too many files".into());
    }
    let metadata = fs::symlink_metadata(source)
        .map_err(|e| format!("Runtime resource {}: {e}", source.display()))?;
    if metadata.file_type().is_symlink() {
        let target = fs::read_link(source).map_err(|e| e.to_string())?;
        safe_link(
            source
                .strip_prefix(root)
                .map_err(|e| e.to_string())?
                .parent()
                .unwrap_or(Path::new("")),
            &target,
        )?;
        if !source
            .canonicalize()
            .map_err(|e| e.to_string())?
            .starts_with(root.canonicalize().map_err(|e| e.to_string())?)
        {
            return Err("Runtime symlink escapes source directory".into());
        }
        std::os::unix::fs::symlink(target, destination).map_err(|e| e.to_string())?;
    } else if metadata.is_dir() {
        fs::create_dir(destination).map_err(|e| e.to_string())?;
        for entry in fs::read_dir(source).map_err(|e| e.to_string())? {
            let entry = entry.map_err(|e| e.to_string())?;
            copy_tree(
                root,
                &entry.path(),
                &destination.join(entry.file_name()),
                budget,
                cancel,
            )?;
        }
    } else if metadata.is_file() {
        budget.bytes = budget
            .bytes
            .checked_add(metadata.len())
            .ok_or("Runtime size overflow")?;
        if budget.bytes > MAX_EXPANDED {
            return Err("Runtime expanded size exceeds limit".into());
        }
        let mut input = File::open(source).map_err(|e| e.to_string())?;
        let mut output = File::create(destination).map_err(|e| e.to_string())?;
        copy_bounded(&mut input, &mut output, metadata.len(), cancel)?;
        use std::os::unix::fs::PermissionsExt as _;
        fs::set_permissions(
            destination,
            fs::Permissions::from_mode(metadata.permissions().mode() & 0o777),
        )
        .map_err(|e| e.to_string())?;
    } else {
        return Err("Unsupported runtime resource type".into());
    }
    Ok(())
}
