# Comprehensive safety status message
Header header

# Overall safety level
uint8 SAFE=0
uint8 WARNING=1
uint8 CRITICAL=2
uint8 EMERGENCY=3
uint8 safety_level

# Hardware health status
uint8 HEALTH_OK=0
uint8 HEALTH_WARNING=1
uint8 HEALTH_CRITICAL=2
uint8 hardware_health
float32 cpu_temperature    # CPU temperature in Celsius
float32 memory_usage      # Memory usage percentage (0-100)

# Sensor status
uint8 SENSOR_OK=0
uint8 SENSOR_DEGRADED=1
uint8 SENSOR_FAILED=2
uint8 camera_status
uint8 imu_status
uint8 encoder_status

# Algorithm performance
float32 lane_detection_confidence
float32 object_detection_fps
float32 control_loop_frequency

# Emergency status
bool emergency_stop_active
string emergency_reason

# Active warnings and alerts
string[] active_warnings
string[] active_alerts

# System health score (0-100)
float32 system_health_score

# Timestamps
time last_safety_check
time last_hardware_check
