# 操作记录

## 2026-05-26 | features-Huang 分支编译

### 步骤

1. 切换到分支：
   ```bash
   git checkout features-Huang
   ```

2. 编译 livox_ros_driver2（需先编译，否则 fast_lio 找不到依赖）：
   ```bash
   cd basic_dev
   source /opt/ros/noetic/setup.bash
   ./src/livox_ros_driver2/build.sh ROS1
   ```

3. 按顺序编译各包（airsim_ros 必须先于 basic_dev/imu_gps_odometry/path_sender，因为它们依赖其生成的头文件）：
   ```bash
   source devel/setup.bash
   catkin_make -DCATKIN_WHITELIST_PACKAGES="airsim_ros"
   catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_lio"
   catkin_make -DCATKIN_WHITELIST_PACKAGES="path_sender"
   catkin_make -DCATKIN_WHITELIST_PACKAGES=""
   ```

4. 修复 ego_planner_node 缺少 OpenCV 链接的问题：
   - 文件：`src/EGO-Planner/src/planner/plan_manage/CMakeLists.txt`
   - 添加 `find_package(OpenCV REQUIRED)`
   - 在 `target_link_libraries(ego_planner_node ...)` 中加入 `${OpenCV_LIBRARIES}`
   - 在 `include_directories(...)` 中加入 `${OpenCV_INCLUDE_DIRS}`

### 注意

- 原因：`plan_env` 的 `grid_map.h` 使用了 `cv::Mat`，但 `plan_manage` 未链接 OpenCV
- 总共 29 个包，全部通过编译

---

## `/waypoints` 发布思路（path_sender）

### 航点构成（三套数据源）

| 数据源 | 说明 |
|---|---|
| `station_[13]` | 12个停机坪坐标，用于起点/终点匹配 |
| `Transit_hub_[13]` | 中转站出入口（高海拔 z≈136~164m） |
| `paths.yaml` | 12条从各站到中转站的密集路径（`path1~path12`） |

### 路径拼接逻辑（`timeCB`，每2秒触发）

```
起点匹配: /airsim_node/initial_pose → 匹配最近 station → initial_num
终点匹配: /airsim_node/end_goal      → 匹配最近 station → end_num

拼装:
  paths[initial_num-1]            ← 起点→中转站入口
+ Transit_hub[initial_num]        ← 中转站入口
+ Transit_hub[end_num]            ← 中转站出口
+ reverse(paths[end_num-1])       ← 中转站出口→终点（反向）
+ end_point[end_num]              ← 桥接点（距站 ~3m，EGO-Planner 1m 容差可达）
```

### 发布策略

- 起点终点均匹配后组装路径，发布一次
- 之后每2秒重发（冗余保证送达）
- `end_goal` 变化时自动重组路径
- 首发后 `initial_path_done=true`，后续旅程将上一终点作为新起点

### 消费者

EGO-Planner FSM 接收 `/waypoints`，z+0.32m 后触发最小 snap 轨迹规划

---

## 恢复 `end_point` 逻辑

### 背景

曾误认为 `end_point[end_num]` 是冗余的"终点延伸点"并删除。经分析发现它是路径拼接的必要桥接段：

| 数据 | ENU 坐标 | 距站3 |
|---|---|---|
| `reverse(path3)` 终点 | (598, -490, 36.8) | ~60m |
| `end_point[3]` | (548, -518, 32.0) | ~3m |
| `station[3]`（AirSim 目标） | (545, -518, 30.2) | — |

### 原因

EGO-Planner 的 `final_goal_` 严格等于 `/waypoints` 最后一个点，到达后 `touch_the_goal` 判定阈值仅 **1m**（`ego_replan_fsm.cpp:249`）。若无 `end_point`，无人机将在 YAML 路径端点停住，距目标尚有 ~60m。`end_point` 将终点从 YAML 路径端点桥接到距站 ~3m 处，EGO-Planner 的 1m 容差可完成最后精调。

### 修改文件

- `src/path_sender/include/path_sender.hpp` — 恢复 `end_point_[13]`、`end_point[13]`、`end_point_NED[13]`
- `src/path_sender/src/path_sender.cpp` — 恢复 `POintSet()` 中的 end_point 赋值、`timeCB()` 中的 `path.emplace_back(end_point[end_num])`

---

## 偏航角修复（traj_server yaw 跟踪）

### 问题

`traj_server.cpp:315` 将偏航角硬编码为起飞时的 `first_yaw_`，全程不更新。`calculate_yaw()` 函数存在且正确（根据速度方向计算 yaw），但调用被注释掉（`traj_server.cpp:348-349`）。

### 修复

- **文件**：`src/EGO-Planner/src/planner/plan_manage/src/traj_server.cpp`
- 在第 300 行新增 `calculate_yaw()` 调用（置于 `PositionCommand` 构造前）
- 第 318 行：`pos_cmd.yaw = first_yaw_` → `pos_cmd.yaw = yaw_yawdot.first`
- 第 319 行：新增 `pos_cmd.yaw_dot = yaw_yawdot.second`
- 移除重复的 `time_last = time_now` 及废弃注释

### 效果

偏航角通过 `atan2(dir.y, dir.x)` 实时跟随速度方向，无人机始终朝向飞行前方。

---

## waypoint 坐标转换与 GPS 噪声抑制

### 问题

`transformWaypointToA()` 每次 replan 时将 waypoint 从 GPS 世界系转换到 FAST-LIO 里程计系：

```cpp
R = q_fastlio × q_gps⁻¹
wp_odom = R × (wp_gps − pos_gps) + pos_fastlio
```

GPS 姿态噪声 ±0.2 rad（~11.5°）导致 `R` 抖动，100m 处 waypoint 偏移可达 ~20m。

### 方案演进

| 分支 | GPS 参考源 | 位置处理 | 朝向处理 |
|---|---|---|---|
| dev2 | `/airsim_node/drone_1/gps` | EMA (α=0.3 固定) | 直接使用 GPS 朝向 |
| dev3 | `/eskf_odom` (IMU+GPS Kalman) | ESKF 已融合，直接赋值 | ESKF 已融合，直接赋值 |
| dev4 | `/airsim_node/drone_1/gps` | EMA (时基+速度自适应) | **SLERP** 球面线性插值 |
| dev4_test | `/airsim_node/drone_1/gps` | EMA (时基+速度自适应) | **FAST-LIO 反推**（当前最优） |

### 当前方案（dev4_test）：FAST-LIO 姿态替代

GPS 系和 FAST-LIO 系都是地球固连系，两者旋转差是**常量**。FAST-LIO 姿态精度 <1°，远优于 GPS 的 ±11.5°。

**① 初始化标定**（gpsPoseCallback，GPS + FAST-LIO 首次同时有效时）：

```
R_world_odom = q_gps_raw × q_fastlio⁻¹
```

**② 每 1s 用 FAST-LIO 推 GPS 朝向**（execFSMCallback 定时器）：

```
q_gps_aligned(t) = R_world_odom × q_fastlio(t)
```

**③ 带入变换矩阵后常数化**：

```
R(t) = q_fastlio(t) × q_gps_aligned(t)⁻¹
     = q_fastlio(t) × (R_world_odom × q_fastlio(t))⁻¹
     = R_world_odom⁻¹   ← 常量，不再抖动
```

两个 `q_fastlio(t)` 抵消后，`transformWaypointToA` 的旋转矩阵变为初始化时标定的一次性常量，GPS 朝向噪声被完全消除。

### 位置处理

位置 XYZ **不**用 FAST-LIO 替代（FAST-LIO 随飞行累积漂移，与 GPS 并非固定偏移）。位置继续用 GPS 的 EMA 自适应滤波（时基 + 速度适应）。

### 修改文件

- `ego_replan_fsm.h` — 新增 `R_world_odom_`（Eigen::Quaterniond）、`R_offset_init_`
- `ego_replan_fsm.cpp:gpsPoseCallback` — 首次 GPS+odom 数据时标定 `R_world_odom_`
- `ego_replan_fsm.cpp:execFSMCallback`（1s 定时器） — 用 FAST-LIO 推 `q_gps_aligned` 更新 `gps_pos_.pose.orientation`

### 附带修复：FAST-LIO 协方差 NaN

`laserMapping.cpp:715` 原在 `publish` 之后才填充协方差，导致 RViz Odometry display 报 NaN 警告。修复：将协方差填充移到 `publish` 之前，对角线设 0.01。
