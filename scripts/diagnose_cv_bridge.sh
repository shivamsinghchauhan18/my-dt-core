#!/usr/bin/env bash

set -euo pipefail

echo "[diag] Python and OpenCV info:"
python3 - <<'PY'
import sys, os
print("python:", sys.executable)
print("version:", sys.version.replace('\n',' '))
try:
    import cv2
    print("cv2 version:", getattr(cv2, '__version__', 'unknown'))
    print("cv2 file:", getattr(cv2, '__file__', 'unknown'))
except Exception as e:
    print("cv2 import error:", e)
try:
    import cv_bridge
    print("cv_bridge file:", cv_bridge.__file__)
except Exception as e:
    print("cv_bridge import error:", e)
PY

echo
CVB_SO="/opt/ros/${ROS_DISTRO:-noetic}/lib/python3/dist-packages/cv_bridge/boost" \
&& CVB_SO=$(ls "$CVB_SO"/cv_bridge_boost*.so 2>/dev/null | head -n1 || true)
if [ -n "$CVB_SO" ]; then
  echo "[diag] ldd on cv_bridge_boost: $CVB_SO"
  ldd "$CVB_SO" || true
else
  echo "[diag] Could not locate cv_bridge_boost .so under /opt/ros/${ROS_DISTRO:-noetic}."
fi

echo
echo "[diag] pip OpenCV packages (if any):"
python3 -m pip freeze | grep -E "^opencv-(python|contrib|python-headless)" || echo "(none)"

echo
echo "[diag] apt OpenCV packages:"
dpkg -l | grep -E "opencv|ros-.*-cv-bridge|python3-opencv" || echo "(none)"

cat <<'EON'

[diag] If cv_bridge import fails or ldd shows missing/not found libs:
  1) Prefer system OpenCV and cv_bridge (apt) over pip wheels:
       python3 -m pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless || true
  2) Ensure apt packages are installed (as root):
       apt-get update && apt-get install -y ros-noetic-cv-bridge python3-opencv
  3) Re-source ROS and workspace, then retry:
       source /opt/ros/noetic/setup.bash
       source /code/catkin_ws/devel/setup.bash
  4) Still failing? Reinstall cv_bridge from apt to repair:
       apt-get install --reinstall -y ros-noetic-cv-bridge
  5) Only as a last resort, build cv_bridge from source matching your OpenCV.

EON
