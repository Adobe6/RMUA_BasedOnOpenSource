#!/usr/bin/env bash
set -euo pipefail

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing command: $1" >&2
    exit 2
  fi
}

section() {
  printf '\n== %s ==\n' "$1"
}

topic_exists() {
  rostopic list | rg -qx "$1"
}

topic_info() {
  local topic="$1"
  section "rostopic info ${topic}"
  if topic_exists "$topic"; then
    rostopic info "$topic"
  else
    echo "MISSING_TOPIC ${topic}"
    return 1
  fi
}

topic_hz() {
  local topic="$1"
  section "rostopic hz ${topic}"
  if ! topic_exists "$topic"; then
    echo "MISSING_TOPIC ${topic}"
    return 1
  fi

  set +e
  timeout 6s rostopic hz "$topic"
  local status=$?
  set -e
  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    echo "HZ_CHECK_FAILED ${topic} status=${status}"
    return "$status"
  fi
}

node_info() {
  local node="$1"
  section "rosnode info ${node}"
  if rosnode list | rg -qx "$node"; then
    rosnode info "$node"
  else
    echo "MISSING_NODE ${node}"
    return 1
  fi
}

need_cmd rostopic
need_cmd rosnode
need_cmd rg
need_cmd timeout

section "topic list cloud/odom"
rostopic list | rg 'cloud_registered|gps_corrected_odometry|^/Odometry$|grid_map/occupancy' || true

fail=0
for topic in \
  /cloud_registered \
  /cloud_registered_corrected \
  /gps_corrected_odometry \
  /Odometry
do
  topic_info "$topic" || fail=1
done

for node in /imu_gps_odometry /drone_0_ego_planner_node; do
  node_info "$node" || fail=1
done

for topic in \
  /cloud_registered \
  /cloud_registered_corrected \
  /gps_corrected_odometry \
  /drone_0_ego_planner_node/grid_map/occupancy \
  /drone_0_ego_planner_node/grid_map/occupancy_inflate
do
  topic_hz "$topic" || fail=1
done

section "expected wiring"
imu_info="$(rosnode info /imu_gps_odometry 2>/dev/null || true)"
ego_info="$(rosnode info /drone_0_ego_planner_node 2>/dev/null || true)"
corrected_info="$(rostopic info /cloud_registered_corrected 2>/dev/null || true)"

if ! topic_exists /cloud_registered_corrected; then
  echo "FAIL: /cloud_registered_corrected is not visible."
  fail=1
fi

if ! awk '
  /^Publishers:/ { in_publishers = 1; next }
  /^Subscribers:/ { in_publishers = 0 }
  in_publishers && /^ \*/ { found = 1 }
  END { exit(found ? 0 : 1) }
' <<<"$corrected_info"; then
  echo "FAIL: /cloud_registered_corrected has no publisher."
  fail=1
fi

if ! grep -q '/cloud_registered_corrected' <<<"$imu_info"; then
  echo "FAIL: /imu_gps_odometry does not show /cloud_registered_corrected in rosnode info."
  fail=1
fi

if ! grep -q '/cloud_registered_corrected' <<<"$ego_info"; then
  echo "FAIL: /drone_0_ego_planner_node is not subscribed to /cloud_registered_corrected."
  fail=1
fi

if [[ "$fail" -eq 0 ]]; then
  echo "OK: corrected cloud pipeline is visible in the live ROS graph."
else
  echo "FAIL: corrected cloud pipeline is incomplete." >&2
fi

exit "$fail"
