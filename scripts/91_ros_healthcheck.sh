#!/usr/bin/env bash

# ROS1 health-check: nodes, pubs/subs, env, and basic lag/rate on selected topics.
# Usage:
#   bash scripts/91_ros_healthcheck.sh [--skip-wtf] [--hz topic1 topic2 ...] [--delay topicA topicB ...]
#
# Notes:
# - This script is safe to run on macOS or the robot.
# - For rate/lag checks, provide specific topics via --hz/--delay to avoid long blocking.
# - Delay checks require messages with std_msgs/Header.

set -euo pipefail

cyan()  { printf "\033[36m%s\033[0m\n" "$*"; }
green() { printf "\033[32m%s\033[0m\n" "$*"; }
yellow(){ printf "\033[33m%s\033[0m\n" "$*"; }
red()   { printf "\033[31m%s\033[0m\n" "$*"; }

SKIP_WTF=0
HZ_TOPICS=()
DELAY_TOPICS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-wtf)
      SKIP_WTF=1; shift ;;
    --hz)
      shift
      while [[ $# -gt 0 && $1 != --* ]]; do HZ_TOPICS+=("$1"); shift; done ;;
    --delay)
      shift
      while [[ $# -gt 0 && $1 != --* ]]; do DELAY_TOPICS+=("$1"); shift; done ;;
    *)
      red "Unknown arg: $1"; exit 2 ;;
  esac
done

need() {
  if ! command -v "$1" >/dev/null 2>&1; then
    red "Missing command: $1. Please ensure ROS1 is installed and your environment is sourced (source devel/setup.bash).";
    exit 1
  fi
}

need rosnode
need rostopic
need rosparam
need rosmsg

cyan "[1/6] Environment" && {
  echo "DATE        : $(date -Iseconds)"
  echo "HOST        : $(hostname)"
  echo "ROS_MASTER  : ${ROS_MASTER_URI:-<unset>}"
  echo "ROS_IP      : ${ROS_IP:-<unset>}"
  echo "ROS_HOSTNAME: ${ROS_HOSTNAME:-<unset>}"
  echo "ROS_DISTRO  : ${ROS_DISTRO:-<unset>}"
  echo
}

cyan "[2/6] Master reachability and namespace" && {
  if rosnode list >/dev/null 2>&1; then
    green "OK: Connected to ROS master"
  else
    red "ERROR: Cannot reach ROS master at \"${ROS_MASTER_URI:-<unset>}\". Is roscore running and network configured?"
    exit 1
  fi
  echo "ROS Namespace: ${ROS_NAMESPACE:-/}"
  echo
}

cyan "[3/6] Nodes status" && {
  nodes=( $(rosnode list || true) )
  if [[ ${#nodes[@]} -eq 0 ]]; then
    yellow "No nodes found. Is anything running?"
  else
    echo "Found ${#nodes[@]} node(s):"
    for n in "${nodes[@]}"; do echo "  - $n"; done
    echo
    echo "Pinging nodes (1 attempt each):"
    for n in "${nodes[@]}"; do
      if rosnode ping -c1 "$n" >/dev/null 2>&1; then
        printf "  %s: " "$n"; green "alive"
      else
        printf "  %s: " "$n"; red "unreachable"
      fi
    done
  fi
  echo
}

cyan "[4/6] Topics with publishers/subscribers" && {
  # rostopic list -v prints each topic with its type and who pubs/subs
  rostopic list -v || true
  echo
}

ros_has_header() {
  local topic="$1"
  local t
  if ! t=$(rostopic type "$topic" 2>/dev/null); then
    return 1
  fi
  if [[ -z "$t" ]]; then
    return 1
  fi
  if rosmsg show "$t" 2>/dev/null | grep -qE '^std_msgs/Header\s+header|^Header\s+header'; then
    return 0
  fi
  return 1
}

ros_now_epoch() {
  # Prints current time seconds.nanoseconds according to use_sim_time
  local use_sim
  use_sim=$(rosparam get use_sim_time 2>/dev/null || echo false)
  if [[ "$use_sim" == "true" ]]; then
    # Get one clock message
    local line
    if line=$(rostopic echo -n1 -p /clock 2>/dev/null | tail -n1); then
      # /clock CSV: secs,nsecs
      local sec nsec
      sec=$(echo "$line" | awk -F',' '{print $1}')
      nsec=$(echo "$line" | awk -F',' '{print $2}')
      python3 - "$sec" "$nsec" <<'PY'
import sys
sec=int(sys.argv[1]); nsec=int(sys.argv[2])
print('{:.9f}'.format(sec + nsec/1e9))
PY
      return 0
    fi
  fi
  # Fallback: wall clock
  python3 - <<'PY'
import time
print('{:.9f}'.format(time.time()))
PY
}

print_delay_for_topic() {
  local topic="$1"
  if ! ros_has_header "$topic"; then
    yellow "  [$topic] No std_msgs/Header; skipping delay"
    return 0
  fi
  local sample
  if ! sample=$(rostopic echo -n1 -p "$topic" 2>/dev/null | tail -n1); then
    yellow "  [$topic] No messages received (timeout); cannot compute delay"
    return 0
  fi
  local sec nsec now delay
  sec=$(echo "$sample" | awk -F',' '{print $2}')
  nsec=$(echo "$sample" | awk -F',' '{print $3}')
  now=$(ros_now_epoch)
  delay=$(python3 - "$sec" "$nsec" "$now" <<'PY'
import sys
sec=float(sys.argv[1]); nsec=float(sys.argv[2]); now=float(sys.argv[3])
stamp=sec + nsec/1e9
print('{:.3f}'.format(now - stamp))
PY
)
  echo "  [$topic] one-shot delay (now - header.stamp): ${delay}s"
}

hz_for_topic() {
  local topic="$1"
  # Best-effort: sample 10 messages or 5s, whichever first, then print avg rate
  # Use python to avoid GNU timeout dependency
  python3 - "$topic" <<'PY'
import sys, subprocess, time, re
topic=sys.argv[1]
cmd=['rostopic','hz','-w','10',topic]
try:
    p=subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
except FileNotFoundError:
    print(f"  [{topic}] rostopic not found")
    sys.exit(0)
start=time.time()
lines=[]
avg=None
pattern=re.compile(r"average rate: ([0-9.]+)")
try:
    while True:
        line=p.stdout.readline()
        if not line:
            if p.poll() is not None:
                break
            time.sleep(0.05)
            if time.time()-start>5.0:
                p.terminate()
                break
            continue
        lines.append(line.rstrip())
        m=pattern.search(line)
        if m:
            avg=m.group(1)
except Exception:
    pass
finally:
    try:
        p.terminate()
    except Exception:
        pass
if avg:
    print(f"  [{topic}] avg rate ~ {avg} Hz")
else:
    print(f"  [{topic}] insufficient samples within 5s")
PY
}

if [[ ${#DELAY_TOPICS[@]} -gt 0 ]]; then
  cyan "[5/6] One-shot delay (Header.stamp) for selected topics"
  for t in "${DELAY_TOPICS[@]}"; do print_delay_for_topic "$t"; done
  echo
fi

if [[ ${#HZ_TOPICS[@]} -gt 0 ]]; then
  cyan "[6/6] Approximate rate for selected topics"
  for t in "${HZ_TOPICS[@]}"; do hz_for_topic "$t"; done
  echo
fi

if [[ $SKIP_WTF -eq 0 ]]; then
  if command -v roswtf >/dev/null 2>&1; then
    cyan "[Extra] roswtf quick diagnostics"
    # roswtf can be slow; run with a short wall-time guard via python wrapper
    python3 - <<'PY'
import subprocess, time, threading, sys
def run():
    try:
        subprocess.run(['roswtf'], check=False)
    except Exception:
        pass
t=threading.Thread(target=run, daemon=True)
t.start()
t.join(timeout=15)
if t.is_alive():
    print('(roswtf timed out after 15s; skipping the rest)')
PY
    echo
  else
    yellow "roswtf not found; skipping"
  fi
fi

green "Done. Tips:"
echo "- To see the full graph visually: rqt_graph"
echo "- To watch live topic stats: rqt_topic"
echo "- To drill into a node: rosnode info /node_name"
echo "- To watch a topic rate interactively: rostopic hz /topic"
