Step-by-step local (no overlay) workflow

- 01_build.sh: Build the catkin workspace in-place using catkin_make. Optional: pass 'clean' to remove build/devel/logs first.
- 02_source.sh: Source ROS and this workspace; prints a quick status.
- 10_camera_decode.sh <veh>: Launch camera decoding.
- 11_camera_rectify.sh <veh>: Launch camera rectification.
- 20_lane_following.sh <veh>: Launch the lane following demo (includes master wiring).
- 30_deadreckoning.sh <veh>: Launch deadreckoning pipeline.
- 40_apriltag_detector.sh <veh>: Launch AprilTag detector pipeline.
- 90_enhanced_autonomous_system.sh <veh>: Launch the enhanced autonomous system demo.

Notes

- Provide the vehicle name either via argument or environment variable VEHICLE_NAME.
- Ensure ROS (Noetic by default) is installed and available at /opt/ros/${ROS_DISTRO}/setup.bash on the target machine.
- If you switch terminals, remember to source again (scripts/02_source.sh or devel/setup.bash).
