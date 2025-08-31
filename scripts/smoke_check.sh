#!/usr/bin/env bash
set -euo pipefail
# Quick smoke checks: syntax and literal topic mapping
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
echo "Repo root: $ROOT"

echo "1) Checking Python syntax (py_compile)"
PY_FILES=$(find "$ROOT" -name '*.py' -not -path '*/.venv/*' -not -path '*/tests/*')
if [ -z "$PY_FILES" ]; then
  echo "No python files found."
else
  python3 -m py_compile $PY_FILES
  echo "Python syntax: OK"
fi

if command -v python3 >/dev/null 2>&1; then
  echo "\n2) Running ros_topic_inspector"
  python3 "$ROOT/src/scripts/ros_topic_inspector.py" || true
else
  echo "python3 not found; skipping ros_topic_inspector"
fi

echo "Smoke checks completed. Note: this doesn't start ROS nodes or validate runtime ROS master."
