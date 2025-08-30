#!/usr/bin/env python3
"""
Restructure repository: move all top-level content into src/ while preserving ROS/Duckietown setup.

Default mode is dry-run. Use --apply to actually move files.

It will:
- Move files/dirs into src/ except a configurable exclusion list
- Update common references in Dockerfile and docs (best-effort)
- Create a backup tarball for rollback

Limitations:
- ROS package discovery may still require updating ROS_PACKAGE_PATH or catkin workspaces
- Launch files and scripts with hard-coded paths may need manual review after the automated pass
"""
import argparse
import os
import shutil
import subprocess
import sys
import tarfile
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

# Items we should not move from repo root
EXCLUDE = {
    ".git", ".github", "src", ".vscode", "__pycache__",
}

# Additional patterns to ignore
EXCLUDE_PREFIXES = {".pytest_cache", "build", "devel", ".mypy_cache"}


def should_skip(p: Path) -> bool:
    name = p.name
    if name in EXCLUDE:
        return True
    if any(name.startswith(pref) for pref in EXCLUDE_PREFIXES):
        return True
    return False


def backup_repo(archive_path: Path):
    with tarfile.open(archive_path, "w:gz") as tar:
        tar.add(REPO_ROOT.as_posix(), arcname=REPO_ROOT.name,
                filter=lambda info: None if info.name.startswith((REPO_ROOT/"src").as_posix()) else info)


def find_top_level_items():
    for child in REPO_ROOT.iterdir():
        if should_skip(child):
            continue
        yield child


def move_items(dry_run: bool):
    moves = []
    for item in find_top_level_items():
        dest = SRC_DIR / item.name
        moves.append((item, dest))
    if dry_run:
        return moves
    for src, dst in moves:
        if dst.exists():
            raise RuntimeError(f"Destination already exists: {dst}")
        shutil.move(src.as_posix(), dst.as_posix())
    return moves


def patch_file(path: Path, replacements):
    try:
        text = path.read_text()
    except Exception:
        return False
    orig = text
    for a, b in replacements:
        text = text.replace(a, b)
    if text != orig:
        path.write_text(text)
        return True
    return False


def update_references():
    # Best-effort: adjust common path references now that root content lives in src/
    replacements = [
        ("src/packages/", "src/src/packages/"),
        ("src/launchers/", "src/src/launchers/"),
        ("src/scripts/", "src/src/scripts/"),
        ("src/docs/", "src/src/docs/"),
        ("src/robot_configs/", "src/src/robot_configs/"),
        ("src/assets/", "src/src/assets/"),
        ("src/tests/", "src/src/tests/"),
    ]
    touched = []
    for root, _, files in os.walk(REPO_ROOT):
        for f in files:
            path = Path(root) / f
            if path.suffix.lower() in {".py", ".sh", ".launch", ".xml", ".md", ".rst", "", ".yaml", ".yml", ".txt"}:
                if patch_file(path, replacements):
                    touched.append(path)
    return touched


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="Apply changes (not just dry-run)")
    args = ap.parse_args()

    SRC_DIR.mkdir(exist_ok=True, parents=True)

    items = list(find_top_level_items())
    print(f"Found {len(items)} items to move under src/:\n  - " + "\n  - ".join(p.name for p in items))

    if not args.apply:
        print("Dry-run only. Use --apply to perform the move.")
        return 0

    backup_name = REPO_ROOT.parent / f"{REPO_ROOT.name}.backup.{datetime.now().strftime('%Y%m%d_%H%M%S')}.tar.gz"
    print(f"Creating backup: {backup_name}")
    backup_repo(backup_name)

    print("Moving items...")
    moves = move_items(dry_run=False)
    for s, d in moves:
        print(f" - {s.name} -> src/{s.name}")

    print("Updating path references (best-effort)...")
    touched = update_references()
    print(f"Patched {len(touched)} files.")

    print("Done. Next steps:\n - Update ROS_PACKAGE_PATH or catkin overlays if needed\n - Fix any remaining hard-coded paths in launch/scripts\n - Rebuild and test")
    return 0


if __name__ == "__main__":
    sys.exit(main())
