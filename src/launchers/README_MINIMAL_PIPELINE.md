Minimal Pipeline launcher

Purpose:
- Start a single-vehicle processing chain: camera -> decoder -> rectifier -> anti_instagram -> line_detector -> lane_filter -> lane_controller

Assumptions:
- You will run this inside the Duckiebot container/environment (e.g., dt-duckiebot-interface) or a host with ROS1 (Noetic) installed.
- `roscore` is running.
- The necessary Python dependencies (opencv, cv_bridge, turbojpeg, numpy, scipy, image_geometry, duckietown_msgs, etc.) are installed inside the container.
- Device or simulated camera is available at `/dev/video0` or you modify the `temp_camera` node to read from a file.

Quick start (inside the container or ROS-enabled shell):

1) Source ROS (if needed) and workspace overlays, then run roscore:

```bash
roscore &
```

2) Launch the minimal pipeline:

```bash
roslaunch src/launchers/minimal_pipeline.launch veh:=blueduckie
```

3) Inspect topics:

```bash
rostopic list
rostopic echo /blueduckie/camera_node/image/compressed
```

Notes and troubleshooting:
- If nodes fail due to missing Python packages (cv2, turbojpeg), install them inside the container.
- Many nodes use parameters; adjust them via rosparam or by editing the launch to pass different params.
- The launch uses package-based `type` and `cwd` to run the Python node scripts in-place. If your ROS environment uses installed packages, replace paths accordingly.

If you want, I can:
- Convert this launch into a set of systemd/docker-compose scripts to start everything in containers.
- Add a fake camera publisher to play back a sample video file (useful when hardware is unavailable).
- Provide a small health-check rosnode that validates end-to-end topic flow.
