#!/usr/bin/env bash
set -euo pipefail

if rg -n "/airsim_node/drone_1/debug/pose_gt|pose_gt" src; then
  echo "Source tree still references pose ground truth." >&2
  exit 1
fi

if rg -n "planNextWaypoint\\(wps_\\[wpt_id_\\]\\)" src/EGO-Planner/src/planner/plan_manage/src/ego_replan_fsm.cpp; then
  echo "First waypoint planning bypasses GPS/odometry frame alignment." >&2
  exit 1
fi

if rg -n "return wp;" src/EGO-Planner/src/planner/plan_manage/src/ego_replan_fsm.cpp; then
  echo "Waypoint frame alignment silently falls back to raw waypoints." >&2
  exit 1
fi

if rg -n "Updated waypoints received, but odom is not ready" src/EGO-Planner/src/planner/plan_manage/src/ego_replan_fsm.cpp; then
  echo "Waypoint updates bypass the shared pending waypoint planner." >&2
  exit 1
fi

if ! rg -n "/gps_corrected_odometry" src/imu_gps_odometry src/EGO-Planner/src/planner/plan_manage/launch/include/advanced_param.xml src/EGO-Planner/src/planner/px4ctrl/launch/ctrl_md.launch src/EGO-Planner/src/planner/plan_manage/src/traj_server.cpp >/dev/null; then
  echo "Control chain is not using GPS-corrected odometry." >&2
  exit 1
fi

if rg -n 'to="/Odometry"|subscribe\("/Odometry"' src/EGO-Planner/src/planner/plan_manage/launch/include/advanced_param.xml src/EGO-Planner/src/planner/px4ctrl/launch/ctrl_md.launch src/EGO-Planner/src/planner/plan_manage/src/traj_server.cpp; then
  echo "Planner, traj server, or px4ctrl still subscribes raw /Odometry." >&2
  exit 1
fi

if ! rg -n "/cloud_registered_corrected" src/imu_gps_odometry src/EGO-Planner/src/planner/plan_manage/launch/include/advanced_param.xml >/dev/null; then
  echo "GPS-corrected planning odometry must use a point cloud in the same corrected frame." >&2
  exit 1
fi

if rg -n '<remap from="~grid_map/cloud" to="/cloud_registered"' src/EGO-Planner/src/planner/plan_manage/launch/include/advanced_param.xml; then
  echo "Planner grid map still consumes raw FAST-LIO cloud while odom is GPS-corrected." >&2
  exit 1
fi

python3 - <<'PY'
import sys
import xml.etree.ElementTree as ET

path = "src/EGO-Planner/src/planner/plan_manage/launch/include/advanced_param.xml"
tree = ET.parse(path)
minimums = {
    "grid_map/obstacles_inflation": 0.40,
    "optimization/obstacle_clearance": 0.80,
    "optimization/obstacle_clearance_soft": 1.20,
}

for name, minimum in minimums.items():
    value_text = None
    for node in tree.iter("param"):
        if node.attrib.get("name") == name:
            value_text = node.attrib.get("value")
            break
    if value_text is None:
        print(f"Missing safety parameter: {name}", file=sys.stderr)
        sys.exit(1)
    value = float(value_text)
    if value < minimum:
        print(f"{name}={value} is below the safety minimum {minimum}.", file=sys.stderr)
        sys.exit(1)
PY

python3 - <<'PY'
import sys
import xml.etree.ElementTree as ET

path = "src/EGO-Planner/src/planner/plan_manage/launch/include/run_in_sim.xml"
tree = ET.parse(path)
maximums = {
    "max_vel": 12.0,
    "max_acc": 8.0,
    "max_jer": 12.0,
}

for name, maximum in maximums.items():
    value_text = None
    for node in tree.iter("arg"):
        if node.attrib.get("name") == name:
            value_text = node.attrib.get("value")
            break
    if value_text is None:
        print(f"Missing dynamic limit arg: {name}", file=sys.stderr)
        sys.exit(1)
    value = float(value_text)
    if value > maximum:
        print(f"{name}={value} is above the safety maximum {maximum}.", file=sys.stderr)
        sys.exit(1)
PY

echo "Source tree does not reference pose ground truth, and control/planning use GPS-corrected odometry and cloud."
