#!/usr/bin/env bash
# Download YOLOv5s model to the expected vehicle_detection path
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
MODEL_REL="src/packages/vehicle_detection/yolov5s.pt"
TARGET_PATH="$REPO_ROOT/$MODEL_REL"
URL_DEFAULT="https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt"

echo "[download_yolo_model] Target: $TARGET_PATH"
mkdir -p "$(dirname "$TARGET_PATH")"

# If already present and reasonably large, skip
if [ -f "$TARGET_PATH" ]; then
  SIZE=$(wc -c < "$TARGET_PATH" | tr -d '[:space:]')
  if [ "${SIZE:-0}" -gt 10000000 ]; then
    echo "[download_yolo_model] Model already present (~$((SIZE/1024/1024)) MB), skipping."
    exit 0
  else
    echo "[download_yolo_model] Existing file too small ($SIZE bytes), re-downloading..."
    rm -f "$TARGET_PATH"
  fi
fi

TMP_FILE="${TARGET_PATH}.tmp"
rm -f "$TMP_FILE"

echo "[download_yolo_model] Downloading yolov5s.pt..."
if command -v curl >/dev/null 2>&1; then
  curl -L -o "$TMP_FILE" "$URL_DEFAULT"
elif command -v wget >/dev/null 2>&1; then
  wget -O "$TMP_FILE" "$URL_DEFAULT"
else
  echo "[download_yolo_model] Neither curl nor wget found. Install one and retry." >&2
  exit 1
fi

if [ ! -f "$TMP_FILE" ]; then
  echo "[download_yolo_model] Download failed (no file)." >&2
  exit 1
fi

SIZE=$(wc -c < "$TMP_FILE" | tr -d '[:space:]')
if [ "${SIZE:-0}" -lt 10000000 ]; then
  echo "[download_yolo_model] Downloaded file too small ($SIZE bytes)." >&2
  rm -f "$TMP_FILE"
  exit 1
fi

mv "$TMP_FILE" "$TARGET_PATH"
echo "[download_yolo_model] Saved model to $TARGET_PATH (~$((SIZE/1024/1024)) MB)."
exit 0
#!/bin/bash
# Script to download YOLOv5 model for enhanced vehicle detection
# This should be run inside the container after workspace setup

set -e

MODEL_URL="https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Function to log with timestamp
log_info() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [INFO] $1"
}

log_error() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [ERROR] $1"
}

log_success() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [SUCCESS] $1"
}

# Determine the correct package directory
PACKAGE_DIRS=(
    "/code/enhanced_ws/src/my-dt-core/src/packages/vehicle_detection"
    "/code/catkin_ws/src/dt-duckiebot-interface/my-dt-core/src/packages/vehicle_detection"
    "$SCRIPT_DIR/../src/packages/vehicle_detection"
)

TARGET_DIR=""
for dir in "${PACKAGE_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        TARGET_DIR="$dir"
        log_info "Found vehicle_detection package at: $TARGET_DIR"
        break
    fi
done

if [ -z "$TARGET_DIR" ]; then
    log_error "Could not find vehicle_detection package directory"
    log_error "Searched in:"
    for dir in "${PACKAGE_DIRS[@]}"; do
        echo "  - $dir"
    done
    exit 1
fi

MODEL_PATH="$TARGET_DIR/yolov5s.pt"

# Check if model already exists
if [ -f "$MODEL_PATH" ]; then
    log_info "YOLOv5 model already exists at: $MODEL_PATH"
    MODEL_SIZE=$(du -h "$MODEL_PATH" | cut -f1)
    log_info "Model size: $MODEL_SIZE"
    exit 0
fi

# Download the model
log_info "Downloading YOLOv5 model..."
log_info "URL: $MODEL_URL"
log_info "Target: $MODEL_PATH"

if command -v wget &> /dev/null; then
    wget -O "$MODEL_PATH" "$MODEL_URL"
elif command -v curl &> /dev/null; then
    curl -L -o "$MODEL_PATH" "$MODEL_URL"
else
    log_error "Neither wget nor curl found. Cannot download model."
    exit 1
fi

# Verify download
if [ -f "$MODEL_PATH" ]; then
    MODEL_SIZE=$(du -h "$MODEL_PATH" | cut -f1)
    log_success "YOLOv5 model downloaded successfully!"
    log_info "Location: $MODEL_PATH"
    log_info "Size: $MODEL_SIZE"
    
    # Set appropriate permissions
    chmod 644 "$MODEL_PATH"
    log_info "Set model file permissions"
else
    log_error "Model download failed"
    exit 1
fi

# Test model loading (optional)
if command -v python3 &> /dev/null; then
    log_info "Testing model loading..."
    python3 -c "
import sys
import os
sys.path.insert(0, '$TARGET_DIR/src')
try:
    from ultralytics import YOLO
    model = YOLO('$MODEL_PATH')
    print('✅ Model loads successfully')
except Exception as e:
    print(f'⚠️  Model loading test failed: {e}')
    print('This may be normal if ultralytics is not installed')
" 2>/dev/null || log_info "Model loading test skipped (dependencies not available)"
fi

log_success "Model setup completed!"
