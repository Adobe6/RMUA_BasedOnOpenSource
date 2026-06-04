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

echo "Source tree does not reference pose ground truth, and control uses GPS-corrected odometry."
