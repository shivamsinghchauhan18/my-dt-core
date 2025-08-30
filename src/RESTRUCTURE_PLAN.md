# Repository Restructure Plan

This repo is a ROS1/duckietown workspace with many hard-coded paths (launch files, catkin/Docker, scripts). Blindly moving everything under `src/` will break imports, ROS package discovery, launchers, docker paths, and docs.

Safer approach:
- Keep top-level files that tooling expects (Dockerfile, Makefile, README, src/docs/, src/launchers/, src/packages/, src/scripts/, src/tests/, etc.) where they are, or update all references in a single atomic commit.
- If you truly need a `src/` monorepo layout, we must:
  1) Move all content, then
  2) Update: ROS_PACKAGE_PATH, launch files, import paths, setup.bash, Docker COPY paths, CI scripts, docs links.

This plan provides a script to simulate and then perform the move. Start with dry-run.

## What will move
Everything except: `.git`, `.github`, `src/` itself, and temporary artifacts under `html/` or `deployment_logs/`. You can edit the exclusion list in the script.

## Steps
1. Run dry-run to preview file moves.
2. If it looks correct, run the real move.
3. Auto-update known references (paths in Dockerfile, docs, and launchers). Review the generated patch report.
4. Build and run quick smoke tests.

## Rollback
The script creates a timestamped backup tarball in the repo root before moving.

