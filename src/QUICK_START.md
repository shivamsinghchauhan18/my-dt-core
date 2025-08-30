# Quick Start - Advanced Autonomous Duckietown System

## 🚀 Ready to Deploy? Follow These Steps:

### 1. Power On Your Robot
- Turn on your DB21J robot "blueduckie"
- Ensure it's connected to your network
- Wait for full boot (LED indicators stable)

### 2. Verify Connectivity
```bash
# Test network connection
ping blueduckie.local

# Test SSH connection
ssh duckie@blueduckie.local "echo 'Robot connected!'"
```

### 3. Deploy the System
```bash
# Run the automated deployment
./src/scripts/deploy_blueduckie.sh
```

### 4. Monitor the System
```bash
# SSH to robot and check status
ssh duckie@blueduckie.local

# Check ROS topics
rostopic list | grep blueduckie

# Monitor performance
htop
```

## 🛡️ Safety First!
- **Always supervise** the robot
- **Test in safe area** first
- **Emergency stop ready**: Press robot button or run:
  ```bash
  rosservice call /blueduckie/fsm_node/set_state "state: 'EMERGENCY_STOP'"
  ```

## 📊 System Status Commands
```bash
# Check system health
rostopic echo /blueduckie/safety_monitor/status -n 1

# View camera feed
rostopic hz /blueduckie/camera_node/image/compressed

# Check FSM state
rostopic echo /blueduckie/fsm_node/mode -n 1

# Monitor detections
rostopic echo /blueduckie/line_detector_node/segment_list -n 1
```

## 🔧 Quick Troubleshooting
| Issue | Solution |
|-------|----------|
| Can't connect to robot | Check network, power, SSH keys |
| High CPU usage | Reduce FPS: `rosparam set /blueduckie/*/target_fps 10` |
| No camera feed | Restart camera: `rosnode kill /blueduckie/camera_node` |
| System slow | Check temperature: `vcgencmd measure_temp` |

## 📱 Key Features Active
- ✅ Adaptive lane detection
- ✅ Enhanced AprilTag stops
- ✅ Object detection & avoidance
- ✅ Safety monitoring
- ✅ Performance optimization
- ⚠️ Lane changing (disabled for safety)

## 🎯 Performance Targets
- **FPS**: 15 (optimized for DB21J)
- **Latency**: <300ms
- **Memory**: <1536MB
- **CPU**: <75%

---
**Need help?** Check `DEPLOYMENT_GUIDE.md` for detailed instructions!