# Minimum Snap - ROS Move Base Global Planner

基于A*路径搜索和Minimum Snap轨迹优化的ROS1 Noetic全局规划器插件，支持设置途径点。

## 项目概述

本项目将Minimum Snap算法实现为ROS move_base的全局规划插件，可以直接替代默认的全局规划器。主要特性包括：

- ✅ **2D路径规划**: 基于A*算法的高效路径搜索
- ✅ **平滑轨迹生成**: 使用Minimum Snap优化生成动态可行的平滑轨迹
- ✅ **多航点支持**: 支持通过多个中间航点进行路径规划
- ✅ **RViz集成**: 提供交互式航点选取工具
- ✅ **Move Base兼容**: 作为插件无缝集成到move_base导航框架

## 系统架构

```
┌─────────────────┐
│  RViz Waypoint  │  ← 用户交互：选择航点
│      Tool       │
└────────┬────────┘
         │ /waypoints
         ↓
┌─────────────────────────────────────────┐
│   Minimum Snap Planner (Plugin)        │
│  ┌───────────────────────────────────┐  │
│  │ 1. 多段A*搜索                      │  │
│  │    起点→航点1→航点2→...→终点      │  │
│  ├───────────────────────────────────┤  │
│  │ 2. 路径点合并与降采样              │  │
│  ├───────────────────────────────────┤  │
│  │ 3. Minimum Snap轨迹优化           │  │
│  │    - 4阶导数最小化                │  │
│  │    - 闭式QP求解                   │  │
│  └───────────────────────────────────┘  │
└────────┬────────────────────────────────┘
         │ smooth trajectory
         ↓
    ┌─────────────┐
    │  Move Base  │  ← 导航框架
    │  + Costmap  │
    └─────────────┘
```

## 模块划分（单包结构）

所有代码已经整合为一个 Catkin 包 `minimum_snap_global_planner`，核心目录如下：

- `src/planner` + `include/minimum_snap_planner`: Move Base 全局规划器插件（`nav_core::BaseGlobalPlanner`），负责多段A*、降采样与Minimum Snap轨迹输出。
- `src/path_searcher` + `include/minimum_snap_planner/path_searcher`: A* 栅格搜索库，可独立复用（含 Map/Subscriber 工具）。
- `src/trajectory` + `include/minimum_snap_planner/trajectory`: Minimum Snap 轨迹生成库（时间分配 + 闭式QP求解）。
- `rviz_tools/waypoint_tool`: RViz 交互式航点工具（`minimum_snap_global_planner/WaypointTool`）。
- `config/`: 规划参数（`planner_params.yaml`）。
- `plugins/`: nav_core & RViz 插件描述文件。

## 安装

### 依赖项

```bash
sudo apt-get install ros-noetic-move-base \
                     ros-noetic-nav-core \
                     ros-noetic-costmap-2d \
                     ros-noetic-dwa-local-planner \
                     ros-noetic-map-server \
                     ros-noetic-rviz \
                     ros-noetic-pcl-ros \
                     qtbase5-dev
```

### 编译

```bash
cd ~/catkin_ws/src
git clone <this-repo>
cd ..
catkin_make
source devel/setup.bash
```

## 使用方法

### 使用步骤

1. **选择航点工具**: 在RViz工具栏中选择 "WaypointSnap"（`minimum_snap_global_planner/WaypointTool`，快捷键：`W`）

2. **添加航点**: 
   - 左键点击地图添加中间航点
   - 航点会显示为绿色球体
   - 可以添加多个航点

3. **设置目标**: 
   - 使用"2D Nav Goal"工具设置最终目标
   - 规划器会自动开始规划

4. **清除航点**: 右键点击清除所有航点

### RViz航点工具详解

| 操作 | 行为 | 说明 |
| ---- | ---- | ---- |
| 左键单击地面 | 添加航点 | 航点以绿色球体显示，并自动附加黄色编号文本 |
| 右键点击航点附近 | 删除最近的单个航点 | 默认 0.5 m 删除半径，可在源码中修改 `delete_tolerance_` |
| 右键点击空白处 | 清空全部航点 | 同时发布空 `PoseArray`，通知规划器重置 |
| 参数 `Feedback Topic` | 默认 `/waypoints/feedback` | 当规划器到达航点后会回传剩余航点，工具自动同步 |
| `Topic` 属性 | 默认 `/waypoints` | 所有新航点以 `geometry_msgs/PoseArray` 发布，供规划器消费 |

**同步机制**  
- 用户添加航点 ⇒ 工具通过 `/waypoints` 发送一次 `PoseArray`。  
- 规划器每次规划后会通过 `/waypoints/feedback` 回传尚未执行的航点列表。  
- 工具订阅反馈并自动重建球体/文本，确保 RViz 显示与实际规划状态一致。  
- 若不希望自动清除，可在 Launch 中关闭反馈订阅或自定义距离阈值。

### 演示

![Planning Demo](doc/planning_demo.gif)

**完整规划流程**：发布多航点 → 设置导航目标 →  Minimum Snap 输出平滑轨迹。

![Waypoint Edit](doc/waypoint_edit.gif)

**航点交互**：展示如何在 RViz 中增删航点（左键添加，右键删除/清空）。

### 规划流程

```
机器人当前位置 → 航点1 → 航点2 → ... → 航点N → 目标点
     ↓              ↓         ↓              ↓         ↓
   A*搜索        A*搜索    A*搜索         A*搜索    A*搜索
     ↓              ↓         ↓              ↓         ↓
     └──────────────┴─────────┴──────────────┴─────────┘
                            ↓
                      合并所有路径点
                            ↓
                      降采样处理
                            ↓
                   Minimum Snap优化
                            ↓
                      平滑轨迹输出
```

## 参数配置

编辑 `minimum_snap_global_planner/config/planner_params.yaml`:

```yaml
MinimumSnapPlanner:
  downsample_resolution: 0.5      # 降采样间隔（米）
  max_vel: 1.0                    # 最大速度（m/s）
  max_accel: 1.0                  # 最大加速度（m/s²）
  polynomial_order: 3             # 多项式阶数（3=jerk最小化）
  trajectory_time_step: 0.01      # 轨迹采样时间步长（秒）
  waypoint_topic: "/waypoints"    # 航点话题名称
```

### 参数说明

| 参数 | 默认值 | 作用 | 调整建议 |
| ---- | ------ | ---- | -------- |
| `downsample_resolution` | 0.5 m | 控制多段 A* 合并后的路径点密度 | 室内推荐 0.3~0.5；室外可增大到 1.0 |
| `max_vel` / `max_accel` | 1.0 m/s，1.0 m/s² | 轨迹时间分配的上限，影响速度轮廓 | 与实际机器人极限一致，适当留裕度 |
| `polynomial_order` | 3 | 最小化的导数阶：2=accel，3=jerk，4=snap | 地面车常用 2/3；飞行器用 3/4 |
| `trajectory_time_step` | 0.01 s | 轨迹离散采样间隔，用于发布 `nav_msgs/Path` | 越小轨迹越平滑，但点数增多 |
| `waypoint_topic` | `/waypoints` | Waypoint Tool 发布的 `PoseArray` 话题 | 如需多个工具，可在 RViz 中修改 |
| `waypoint_feedback_topic` | `/waypoints/feedback` | 规划器回传剩余航点列表 | 留空则不回传，航点需手动清除 |
| `robot_pose_topic` / `robot_pose_type` | `/amcl_pose`, `pose_with_covariance` | 机器人位姿来源（AMCL/odom等） | 若改用 TF，可将话题留空并手动调用 `costmap_ros_->getRobotPose()` |
| `waypoint_reached_tolerance` | 0.35 m | 判定机器人已到达当前航点的距离阈值 | 根据定位精度调整，过小可能导致无法清空 |

> **提示**：`planner_params.yaml` 中的所有参数会被加载到 `~MinimumSnapPlanner` 命名空间，可在 launch 中通过 `rosparam` 覆盖。

## 在自己的项目中使用

### 1. 配置move_base

在你的launch文件中：

```xml
<node pkg="move_base" type="move_base" name="move_base">
  <!-- 设置全局规划器 -->
  <param name="base_global_planner" value="minimum_snap_planner/MinimumSnapPlanner"/>
  
  <!-- 加载参数 -->
  <rosparam file="$(find minimum_snap_global_planner)/config/planner_params.yaml" command="load"/>
  
  <!-- 你的costmap配置 -->
  <rosparam file="$(find your_package)/config/costmap_common.yaml" command="load" ns="global_costmap"/>
  <rosparam file="$(find your_package)/config/global_costmap.yaml" command="load"/>
  <rosparam file="$(find your_package)/config/local_costmap.yaml" command="load"/>
</node>
```

### 2. 在RViz中启用航点工具

启动 RViz，并从工具栏添加 `WaypointSnap`（对应 `minimum_snap_global_planner/WaypointTool`）。

## 话题接口

### 订阅
- `/waypoints` (geometry_msgs/PoseArray): 中间航点
- `/move_base_simple/goal` (geometry_msgs/PoseStamped): 最终目标（由move_base处理）
- Costmap相关话题（由move_base提供）

### 发布
- `~MinimumSnapPlanner/global_plan` (nav_msgs/Path): 生成的全局路径（用于可视化）

## 算法细节

### A*搜索
- 基于栅格的图搜索
- 支持8方向移动（可配置）
- 使用曼哈顿距离启发式
- 带tie-breaking避免路径震荡

### Minimum Snap优化
- 目标：最小化轨迹的snap（4阶导数）
- 方法：闭式QP求解（无需迭代）
- 约束：位置、速度、加速度在航点处连续
- 输出：分段多项式轨迹

数学表达：
```
min ∫ ||d⁴r/dt⁴||² dt
s.t. r(0) = p₀, r(T) = pₙ
     r(tᵢ) = pᵢ (航点约束)
     连续性约束
```

## 致谢

原始Minimum Snap实现：
```
Zhang Zhimeng
https://blog.csdn.net/u011341856/article/details/121861930
```

## 许可证

BSD License

## 更新日志

### v1.0.0 (2024)
- ✅ 删除3D规划部分
- ✅ 实现move_base全局规划器插件
- ✅ 添加RViz航点选取工具
- ✅ 支持多段A*+Minimum Snap规划
- ✅ 从costmap获取障碍物信息
- ✅ 路径降采样功能
- ✅ 完整的配置文件和文档



