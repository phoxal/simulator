#!/usr/bin/env python3
"""Package an already built simulator with its pinned native macOS framework."""

import argparse
import hashlib
import json
from pathlib import Path
import plistlib
import re
import shutil
import subprocess
import tempfile


def digest(path):
    hasher = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def run(*args):
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f"{args[0]} failed ({result.returncode}): {result.stdout}")
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--mujoco-distribution", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--version", required=True)
    args = parser.parse_args()
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", args.version):
        parser.error("--version must be a three-part numeric application version")
    output = args.output.absolute()
    receipt = output.with_suffix(".app.provenance.json")
    if (output.suffix != ".app" or output.exists() or output.is_symlink()
            or receipt.exists() or receipt.is_symlink()):
        parser.error("--output must name a new .app directory")
    binary = args.binary.resolve(strict=True)
    distribution = args.mujoco_distribution.resolve(strict=True)
    native_name = "libmujoco.3.12.0.dylib"
    native = distribution / "mujoco.framework/Versions/A" / native_name
    dependency = "@rpath/mujoco.framework/Versions/A/" + native_name
    if dependency not in run("otool", "-L", str(binary)):
        parser.error("binary does not link the supported MuJoCo 3.12.0 framework")
    for required in [native, distribution / "LICENSE", distribution / "THIRD_PARTY_NOTICES"]:
        if not required.is_file():
            parser.error(f"native distribution is incomplete: {required}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".phoxal-package-", dir=output.parent) as stage:
        app = Path(stage) / output.name
        contents = app / "Contents"
        executable = contents / "MacOS/phoxal-simulator-mujoco"
        resources = contents / "Resources"
        executable.parent.mkdir(parents=True)
        resources.mkdir()
        shutil.copy2(binary, executable)
        frameworks = contents / "Frameworks"
        frameworks.mkdir()
        bundled_native = frameworks / native_name
        shutil.copy2(native, bundled_native)
        for notice in ["LICENSE", "THIRD_PARTY_NOTICES"]:
            shutil.copy2(distribution / notice, resources / ("MUJOCO_" + notice))
        shutil.copy2(Path(__file__).resolve().parent.parent / "LICENSE", resources / "LICENSE")
        with (contents / "Info.plist").open("wb") as plist:
            plistlib.dump({
                "CFBundleExecutable": executable.name,
                "CFBundleIdentifier": "com.phoxal.simulator",
                "CFBundleName": "Phoxal Simulator",
                "CFBundlePackageType": "APPL",
                "CFBundleShortVersionString": args.version,
                "CFBundleVersion": args.version,
                "NSHighResolutionCapable": True,
            }, plist)
        run("install_name_tool", "-add_rpath", "@executable_path/../Frameworks", str(executable))
        run("install_name_tool", "-change", dependency, "@rpath/" + native_name, str(executable))
        # Keep the upstream library signature and bytes intact. Only the app
        # and its changed executable need a local ad-hoc signature.
        run("codesign", "--force", "--sign", "-", str(executable))
        if digest(native) != digest(bundled_native):
            raise RuntimeError("native library changed during packaging")
        provenance = {
            "application_version": args.version,
            "application_input_sha256": digest(binary),
            "mujoco_version": "3.12.0",
            "mujoco_library_sha256": digest(bundled_native),
            "signing": "ad-hoc",
            "notarized": False,
        }
        (resources / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
        run("codesign", "--force", "--sign", "-", str(app))
        run("codesign", "--verify", "--deep", "--strict", str(app))
        # The system Python process does not preserve DYLD_LIBRARY_PATH. This
        # deliberately checks the same application-relative loader path callers use.
        run(str(executable), "--help")
        # The final executable seals the resource manifest in its signature.
        # Its own hash therefore belongs outside that signed resource manifest.
        provenance["application_packaged_sha256"] = digest(executable)
        app.rename(output)
        with receipt.open("x") as record:
            record.write(json.dumps(provenance, indent=2) + "\n")
    print(output)


if __name__ == "__main__":
    main()
