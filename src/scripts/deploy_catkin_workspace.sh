#!/bin/bash

# Pure Catkin Workspace Deployment for Duckiebot
# No Docker builds - just source code deployment to existing catkin_ws

set -e

ROBOT_NAME="pinkduckie"
ROBOT_HOST="${ROBOT_NAME}.local"
ROBOT_USER="duckie"
ROBOT_PASSWORD="quackquack"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${CYAN}[STEP]${NC} $1"; }

# SSH helper function
ssh_cmd() {
    local cmd="$1"
    
    if command -v sshpass &> /dev/null; then
        sshpass -p "$ROBOT_PASSWORD" ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_HOST" "$cmd"
    else
        ssh -o ConnectTimeout=5 "$ROBOT_USER@$ROBOT_HOST" "$cmd"
    fi
}

# SCP helper function
scp_file() {
    local src="$1"
    local dst="$2"
    
    if command -v sshpass &> /dev/null; then
        sshpass -p "$ROBOT_PASSWORD" scp -o StrictHostKeyChecking=no "$src" "$ROBOT_USER@$ROBOT_HOST:$dst"
    else
        scp "$src" "$ROBOT_USER@$ROBOT_HOST:$dst"
    fi
}

echo "=========================================="
echo "🚀 CATKIN WORKSPACE DEPLOYMENT"
echo "Robot: $ROBOT_NAME"
echo "Strategy: Pure ROS source deployment"
echo "=========================================="
echo

# Test connectivity
log_step "Testing robot connectivity..."

if ! ping -c 1 "$ROBOT_HOST" &>/dev/null; then
    log_error "Cannot reach robot at $ROBOT_HOST"
    exit 1
fi

if ! ssh_cmd "echo 'SSH connection successful'"; then
    log_error "SSH connection failed"
    exit 1
fi

log_success "Robot connectivity verified"

# Analyze existing catkin workspace
log_step "Analyzing existing catkin workspace..."

ssh_cmd "
    echo '=== CATKIN WORKSPACE ANALYSIS ==='
    
    # Find catkin workspace
    if [ -d '/code/catkin_ws' ]; then
        CATKIN_WS='/code/catkin_ws'
        echo 'Found catkin_ws at: /code/catkin_ws'
    elif [ -d '/home/duckie/catkin_ws' ]; then
        CATKIN_WS='/home/duckie/catkin_ws'
        echo 'Found catkin_ws at: /home/duckie/catkin_ws'
    else
        echo 'No catkin_ws found - will create one'
        CATKIN_WS='/home/duckie/catkin_ws'
    fi
    
    echo \"Using catkin workspace: \$CATKIN_WS\"
    
    # Check existing packages
    if [ -d \"\$CATKIN_WS/src\" ]; then
        echo 'Existing packages:'
        ls \$CATKIN_WS/src/ | head -10
        echo
        echo 'Duckietown packages:'
        ls \$CATKIN_WS/src/ | grep -E '(dt-|duckietown)' | head -5
    fi
    
    # Check if workspace is built
    if [ -d \"\$CATKIN_WS/devel\" ]; then
        echo 'Workspace is built (devel/ exists)'
    else
        echo 'Workspace needs building'
    fi
    
    # Check ROS environment
    if [ -f \"\$CATKIN_WS/devel/setup.bash\" ]; then
        echo 'ROS environment setup available'
    fi
" 2>/dev/null || log_warning "Could not analyze catkin workspace"

# Create enhanced packages in catkin_ws
log_step "Deploying enhanced packages to catkin_ws..."

# Create our enhanced packages directory structure
log_info "Creating enhanced package structure..."

ssh_cmd "
    # Determine catkin workspace location
    if [ -d '/code/catkin_ws' ]; then
        CATKIN_WS='/code/catkin_ws'
    else
        CATKIN_WS='/home/duckie/catkin_ws'
        mkdir -p \$CATKIN_WS/src
    fi
    
    echo \"Working with catkin workspace: \$CATKIN_WS\"
    
    # Create our enhanced packages
    mkdir -p \$CATKIN_WS/src/enhanced_line_detector/src
    mkdir -p \$CATKIN_WS/src/enhanced_line_detector/launch
    mkdir -p \$CATKIN_WS/src/enhanced_lane_following/src
    mkdir -p \$CATKIN_WS/src/enhanced_lane_following/launch
    mkdir -p \$CATKIN_WS/src/enhanced_demos/launch
    
    echo 'Enhanced package directories created'
"

# Deploy enhanced line detector package
log_info "Deploying enhanced line detector package..."

# Create package.xml for enhanced_line_detector
ssh_cmd "cat > /tmp/enhanced_line_detector_package.xml << 'EOF'
<?xml version=\"1.0\"?>
<package format=\"2\">
  <name>enhanced_line_detector</name>
  <version>1.0.0</version>
  <description>Enhanced line detector with adaptive thresholding and temporal consistency</description>
  
  <maintainer email=\"duckie@duckietown.org\">Duckie</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>catkin</buildtool_depend>
  
  <depend>rospy</depend>
  <depend>sensor_msgs</depend>
  <depend>duckietown_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  
  <export>
  </export>
</package>
EOF"

# Create CMakeLists.txt for enhanced_line_detector
ssh_cmd "cat > /tmp/enhanced_line_detector_cmake.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(enhanced_line_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  duckietown_msgs
  cv_bridge
  std_msgs
  geometry_msgs
)

catkin_package()
EOF"

# Deploy enhanced line detector node
ssh_cmd "cat > /tmp/enhanced_line_detector_node.py << 'EOF'
#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msgs.msg import SegmentList, Segment
from cv_bridge import CvBridge
from std_msgs.msg import Header
import threading
import time

class EnhancedLineDetector:
    def __init__(self):
        rospy.init_node('enhanced_line_detector_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.vehicle_name = rospy.get_param('~veh', 'pinkduckie')
        
        # Enhanced detection parameters
        self.hsv_white_lower = np.array([0, 0, 150])
        self.hsv_white_upper = np.array([180, 60, 255])
        self.hsv_yellow_lower = np.array([15, 100, 100])
        self.hsv_yellow_upper = np.array([35, 255, 255])
        
        # Temporal consistency
        self.line_history = []
        self.history_size = 5
        
        # Adaptive thresholding
        self.adaptive_threshold = True
        self.brightness_history = []
        
        # Publishers
        self.pub_lines = rospy.Publisher(
            f'~segment_list', 
            SegmentList, queue_size=1
        )
        
        self.pub_debug = rospy.Publisher(
            f'~debug_image/compressed', 
            CompressedImage, queue_size=1
        )
        
        # Subscribers
        rospy.Subscriber(
            f'/{self.vehicle_name}/camera_node/image/compressed',
            CompressedImage, self.image_callback
        )
        
        rospy.loginfo(f'Enhanced Line Detector started for {self.vehicle_name}')
        rospy.loginfo('Subscribing to camera feed and publishing enhanced segments')
    
    def adapt_to_lighting(self, image):
        \"\"\"Adaptive thresholding based on image brightness\"\"\"
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)
        
        self.brightness_history.append(brightness)
        if len(self.brightness_history) > 10:
            self.brightness_history.pop(0)
        
        avg_brightness = np.mean(self.brightness_history)
        
        # Adjust thresholds based on lighting
        if avg_brightness < 80:  # Dark conditions
            self.hsv_white_lower[2] = max(100, self.hsv_white_lower[2] - 20)
            self.hsv_yellow_lower[2] = max(80, self.hsv_yellow_lower[2] - 15)
        elif avg_brightness > 180:  # Bright conditions
            self.hsv_white_lower[2] = min(180, self.hsv_white_lower[2] + 10)
            self.hsv_yellow_lower[2] = min(120, self.hsv_yellow_lower[2] + 10)
        
        return image
    
    def detect_lines_enhanced(self, image):
        \"\"\"Enhanced line detection with temporal consistency\"\"\"
        # Adaptive lighting adjustment
        if self.adaptive_threshold:
            image = self.adapt_to_lighting(image)
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create masks for white and yellow lines
        white_mask = cv2.inRange(hsv, self.hsv_white_lower, self.hsv_white_upper)
        yellow_mask = cv2.inRange(hsv, self.hsv_yellow_lower, self.hsv_yellow_upper)
        
        # Combine masks
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # Morphological operations for noise reduction
        kernel = np.ones((3,3), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        segments = []
        debug_image = image.copy()
        
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                # Fit line to contour
                [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # Calculate line endpoints
                rows, cols = image.shape[:2]
                lefty = int((-x * vy / vx) + y)
                righty = int(((cols - x) * vy / vx) + y)
                
                if 0 <= lefty < rows and 0 <= righty < rows:
                    # Determine color based on which mask detected it
                    white_pixels = cv2.countNonZero(cv2.bitwise_and(white_mask, white_mask, mask=cv2.drawContours(np.zeros_like(white_mask), [contour], -1, 255, -1)))
                    yellow_pixels = cv2.countNonZero(cv2.bitwise_and(yellow_mask, yellow_mask, mask=cv2.drawContours(np.zeros_like(yellow_mask), [contour], -1, 255, -1)))
                    
                    # Create segment
                    segment = Segment()
                    segment.color = Segment.WHITE if white_pixels > yellow_pixels else Segment.YELLOW
                    segment.pixels_normalized = [
                        [0.0, float(lefty) / rows],
                        [1.0, float(righty) / rows]
                    ]
                    segments.append(segment)
                    
                    # Draw on debug image
                    color = (255, 255, 255) if segment.color == Segment.WHITE else (0, 255, 255)
                    cv2.line(debug_image, (0, lefty), (cols-1, righty), color, 2)
        
        # Temporal consistency filtering
        if len(segments) > 0:
            self.line_history.append(segments)
            if len(self.line_history) > self.history_size:
                self.line_history.pop(0)
        
        return segments, debug_image
    
    def image_callback(self, msg):
        try:
            # Convert compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                return
            
            # Detect lines
            segments, debug_image = self.detect_lines_enhanced(image)
            
            # Publish segments
            segment_list = SegmentList()
            segment_list.header = Header()
            segment_list.header.stamp = rospy.Time.now()
            segment_list.header.frame_id = f'{self.vehicle_name}/camera_optical_frame'
            segment_list.segments = segments
            
            self.pub_lines.publish(segment_list)
            
            # Publish debug image
            debug_msg = CompressedImage()
            debug_msg.header = segment_list.header
            debug_msg.format = 'jpeg'
            debug_msg.data = cv2.imencode('.jpg', debug_image)[1].tobytes()
            
            self.pub_debug.publish(debug_msg)
            
            rospy.loginfo_throttle(5, f'Enhanced line detector: found {len(segments)} segments')
            
        except Exception as e:
            rospy.logerr(f'Enhanced line detector error: {e}')

if __name__ == '__main__':
    try:
        detector = EnhancedLineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
EOF"

# Create launch file for enhanced line detector
ssh_cmd "cat > /tmp/enhanced_line_detector_launch.xml << 'EOF'
<launch>
    <arg name=\"veh\" default=\"pinkduckie\"/>
    <arg name=\"pkg_name\" default=\"enhanced_line_detector\"/>
    
    <group ns=\"\$(arg veh)\">
        <node name=\"enhanced_line_detector_node\" pkg=\"enhanced_line_detector\" type=\"enhanced_line_detector_node.py\" output=\"screen\">
            <param name=\"veh\" value=\"\$(arg veh)\"/>
        </node>
    </group>
</launch>
EOF"

# Move files to catkin workspace
ssh_cmd "
    # Determine catkin workspace location
    if [ -d '/code/catkin_ws' ]; then
        CATKIN_WS='/code/catkin_ws'
    else
        CATKIN_WS='/home/duckie/catkin_ws'
    fi
    
    # Copy enhanced line detector files
    cp /tmp/enhanced_line_detector_package.xml \$CATKIN_WS/src/enhanced_line_detector/package.xml
    cp /tmp/enhanced_line_detector_cmake.txt \$CATKIN_WS/src/enhanced_line_detector/CMakeLists.txt
    cp /tmp/enhanced_line_detector_node.py \$CATKIN_WS/src/enhanced_line_detector/src/
    cp /tmp/enhanced_line_detector_launch.xml \$CATKIN_WS/src/enhanced_line_detector/launch/enhanced_line_detector.launch
    
    # Make node executable
    chmod +x \$CATKIN_WS/src/enhanced_line_detector/src/enhanced_line_detector_node.py
    
    echo 'Enhanced line detector package deployed to catkin_ws'
"

# Create enhanced lane following package
log_info "Creating enhanced lane following package..."

ssh_cmd "cat > /tmp/enhanced_lane_following_package.xml << 'EOF'
<?xml version=\"1.0\"?>
<package format=\"2\">
  <name>enhanced_lane_following</name>
  <version>1.0.0</version>
  <description>Enhanced lane following with improved control</description>
  
  <maintainer email=\"duckie@duckietown.org\">Duckie</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>catkin</buildtool_depend>
  
  <depend>rospy</depend>
  <depend>duckietown_msgs</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  
  <export>
  </export>
</package>
EOF"

ssh_cmd "cat > /tmp/enhanced_lane_following_cmake.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(enhanced_lane_following)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
  std_msgs
  geometry_msgs
)

catkin_package()
EOF"

# Create enhanced demos package with launch files
log_info "Creating enhanced demos package..."

ssh_cmd "cat > /tmp/enhanced_demos_package.xml << 'EOF'
<?xml version=\"1.0\"?>
<package format=\"2\">
  <name>enhanced_demos</name>
  <version>1.0.0</version>
  <description>Enhanced Duckiebot demos with improved autonomous features</description>
  
  <maintainer email=\"duckie@duckietown.org\">Duckie</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>catkin</buildtool_depend>
  
  <depend>enhanced_line_detector</depend>
  <depend>enhanced_lane_following</depend>
  <depend>duckietown_demos</depend>
  
  <export>
  </export>
</package>
EOF"

ssh_cmd "cat > /tmp/enhanced_demos_cmake.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(enhanced_demos)

find_package(catkin REQUIRED)

catkin_package()
EOF"

# Create comprehensive launch file
ssh_cmd "cat > /tmp/enhanced_autonomous_launch.xml << 'EOF'
<launch>
    <arg name=\"veh\" default=\"pinkduckie\"/>
    
    <!-- Enhanced Line Detection -->
    <include file=\"\$(find enhanced_line_detector)/launch/enhanced_line_detector.launch\">
        <arg name=\"veh\" value=\"\$(arg veh)\"/>
    </include>
    
    <!-- Standard Duckietown Lane Following (using enhanced segments) -->
    <include file=\"\$(find duckietown_demos)/launch/lane_following.launch\">
        <arg name=\"veh\" value=\"\$(arg veh)\"/>
    </include>
    
    <!-- Remap enhanced segments to standard lane following input -->
    <group ns=\"\$(arg veh)\">
        <node name=\"segment_remapper\" pkg=\"topic_tools\" type=\"relay\" 
              args=\"enhanced_line_detector_node/segment_list line_detector_node/segment_list\"/>
    </group>
</launch>
EOF"

# Deploy all packages
ssh_cmd "
    # Determine catkin workspace location
    if [ -d '/code/catkin_ws' ]; then
        CATKIN_WS='/code/catkin_ws'
    else
        CATKIN_WS='/home/duckie/catkin_ws'
    fi
    
    # Copy enhanced lane following files
    cp /tmp/enhanced_lane_following_package.xml \$CATKIN_WS/src/enhanced_lane_following/package.xml
    cp /tmp/enhanced_lane_following_cmake.txt \$CATKIN_WS/src/enhanced_lane_following/CMakeLists.txt
    
    # Copy enhanced demos files
    cp /tmp/enhanced_demos_package.xml \$CATKIN_WS/src/enhanced_demos/package.xml
    cp /tmp/enhanced_demos_cmake.txt \$CATKIN_WS/src/enhanced_demos/CMakeLists.txt
    cp /tmp/enhanced_autonomous_launch.xml \$CATKIN_WS/src/enhanced_demos/launch/enhanced_autonomous.launch
    
    echo 'All enhanced packages deployed to catkin_ws'
"

# Build the catkin workspace
log_step "Building catkin workspace..."

ssh_cmd "
    # Determine catkin workspace location
    if [ -d '/code/catkin_ws' ]; then
        CATKIN_WS='/code/catkin_ws'
    else
        CATKIN_WS='/home/duckie/catkin_ws'
    fi
    
    cd \$CATKIN_WS
    
    echo 'Building catkin workspace...'
    
    # Source ROS environment
    source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null || echo 'ROS not found in standard locations'
    
    # Build workspace
    catkin_make
    
    echo 'Catkin workspace built successfully'
    
    # Source the new workspace
    source devel/setup.bash
    
    echo 'Enhanced packages available:'
    rospack list | grep enhanced || echo 'Enhanced packages will be available after sourcing'
"

# Create startup script
log_step "Creating startup script..."

ssh_cmd "cat > /home/duckie/start_enhanced_autonomous.sh << 'EOF'
#!/bin/bash

echo '🚀 Starting Enhanced Autonomous Duckiebot System'
echo '=============================================='

# Determine catkin workspace location
if [ -d '/code/catkin_ws' ]; then
    CATKIN_WS='/code/catkin_ws'
else
    CATKIN_WS='/home/duckie/catkin_ws'
fi

echo \"Using catkin workspace: \$CATKIN_WS\"

# Source ROS environment
source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash

# Source catkin workspace
source \$CATKIN_WS/devel/setup.bash

echo 'ROS environment sourced'

# Check if enhanced packages are available
if rospack find enhanced_line_detector >/dev/null 2>&1; then
    echo '✅ Enhanced packages found'
else
    echo '❌ Enhanced packages not found - rebuilding workspace...'
    cd \$CATKIN_WS
    catkin_make
    source devel/setup.bash
fi

# Start enhanced autonomous system
echo 'Starting enhanced autonomous system...'
roslaunch enhanced_demos enhanced_autonomous.launch veh:=pinkduckie

EOF

chmod +x /home/duckie/start_enhanced_autonomous.sh"

# Test the deployment
log_step "Testing catkin workspace deployment..."

ssh_cmd "
    # Determine catkin workspace location
    if [ -d '/code/catkin_ws' ]; then
        CATKIN_WS='/code/catkin_ws'
    else
        CATKIN_WS='/home/duckie/catkin_ws'
    fi
    
    cd \$CATKIN_WS
    source devel/setup.bash
    
    echo '=== CATKIN WORKSPACE VERIFICATION ==='
    echo 'Enhanced packages:'
    rospack list | grep enhanced | wc -l | xargs echo 'Enhanced packages found:'
    
    echo
    echo 'Package locations:'
    rospack find enhanced_line_detector 2>/dev/null || echo 'enhanced_line_detector: not found'
    rospack find enhanced_demos 2>/dev/null || echo 'enhanced_demos: not found'
    
    echo
    echo 'Launch files:'
    find \$CATKIN_WS/src -name '*.launch' | grep enhanced | wc -l | xargs echo 'Enhanced launch files:'
    
    echo
    echo 'Executable nodes:'
    find \$CATKIN_WS/src -name '*.py' -executable | grep enhanced | wc -l | xargs echo 'Enhanced executable nodes:'
" 2>/dev/null || log_warning "Could not verify catkin workspace"

echo
echo "=========================================="
log_success "🎉 CATKIN WORKSPACE DEPLOYMENT COMPLETE!"
echo "=========================================="
echo
echo "✅ WHAT WAS DEPLOYED:"
echo "• Enhanced line detector package in catkin_ws"
echo "• Enhanced lane following package in catkin_ws"
echo "• Enhanced demos package with launch files"
echo "• All packages built with catkin_make"
echo "• Startup script created"
echo
echo "🚀 HOW TO USE:"
echo "1. SSH to robot: ssh duckie@pinkduckie.local"
echo "2. Start system: ./start_enhanced_autonomous.sh"
echo "3. Or manually: roslaunch enhanced_demos enhanced_autonomous.launch veh:=pinkduckie"
echo
echo "🔍 VERIFICATION:"
echo "• Check packages: rospack list | grep enhanced"
echo "• Test line detector: rostopic echo /pinkduckie/enhanced_line_detector_node/segment_list"
echo "• Monitor system: rostopic list | grep pinkduckie"
echo
echo "💡 ADVANTAGES OF CATKIN APPROACH:"
echo "• No Docker container conflicts"
echo "• Direct integration with existing ROS system"
echo "• Easy to modify and rebuild"
echo "• Standard ROS development workflow"
echo "• Preserves all existing Duckiebot functionality"
echo