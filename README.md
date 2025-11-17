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

## 功能包说明

### 1. `minimum_snap_planner`
核心全局规划器插件，实现了`nav_core::BaseGlobalPlanner`接口。

**主要功能**:
- 从costmap获取障碍物信息
- 多段A*路径搜索
- 路径降采样
- Minimum Snap轨迹优化
- 发布平滑轨迹给move_base

### 2. `waypoint_rviz_tools`
RViz 可视化插件，提供交互式航点选取工具。

**功能**:
- 左键点击添加航点
- 右键清除所有航点
- 实时可视化航点（绿色球体）
- 发布航点到`/waypoints`话题

### 3. `path_searcher`
A*路径搜索库。

**特性**:
- 2D栅格地图搜索
- 支持对角线移动
- 曼哈顿启发式函数
- 超时保护机制

### 4. `trajectory_generator`
Minimum Snap轨迹生成库。

**特性**:
- 闭式QP求解
- 多项式轨迹表示
- 时间自动分配
- 支持位置、速度、加速度约束

## 安装

### 依赖项

```bash
sudo apt-get install ros-noetic-move-base \
                     ros-noetic-nav-core \
                     ros-noetic-costmap-2d \
                     ros-noetic-dwa-local-planner \
                     ros-noetic-map-server
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

1. **选择航点工具**: 在RViz工具栏中选择 "WaypointSnap"（`waypoint_rviz_tools/WaypointTool`，快捷键：`W`）

2. **添加航点**: 
   - 左键点击地图添加中间航点
   - 航点会显示为绿色球体
   - 可以添加多个航点

3. **设置目标**: 
   - 使用"2D Nav Goal"工具设置最终目标
   - 规划器会自动开始规划

4. **清除航点**: 右键点击清除所有航点

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

编辑 `minimum_snap_planner/config/planner_params.yaml`:

```yaml
WaypointSnapPlanner:
  downsample_resolution: 0.5    # 降采样间隔（米）
  max_vel: 1.0                  # 最大速度（m/s）
  max_accel: 1.0                # 最大加速度（m/s²）
  polynomial_order: 3           # 多项式阶数（3=jerk最小化）
  trajectory_time_step: 0.01    # 轨迹采样时间步长（秒）
  waypoint_topic: "/waypoints"  # 航点话题名称
```

### 参数说明

- **downsample_resolution**: 控制路径点密度，值越小轨迹越精细但计算量越大
- **max_vel/max_accel**: 用于时间分配，影响轨迹的动态特性
- **polynomial_order**: 
  - 2: 最小化加速度
  - 3: 最小化jerk（加加速度）
  - 4: 最小化snap（四阶导数，适合四旋翼）

## 在自己的项目中使用

### 1. 配置move_base

在你的launch文件中：

```xml
<node pkg="move_base" type="move_base" name="move_base">
  <!-- 设置全局规划器 -->
  <param name="base_global_planner" value="minimum_snap_planner/WaypointSnapPlanner"/>
  
  <!-- 加载参数 -->
  <rosparam file="$(find minimum_snap_planner)/config/planner_params.yaml" command="load"/>
  
  <!-- 你的costmap配置 -->
  <rosparam file="$(find your_package)/config/costmap_common.yaml" command="load" ns="global_costmap"/>
  <rosparam file="$(find your_package)/config/global_costmap.yaml" command="load"/>
  <rosparam file="$(find your_package)/config/local_costmap.yaml" command="load"/>
</node>
```

### 2. 在RViz中启用航点工具

启动 RViz，并从工具栏添加 `WaypointSnap`（对应 `waypoint_rviz_tools/WaypointTool`）。

## 话题接口

### 订阅
- `/waypoints` (geometry_msgs/PoseArray): 中间航点
- `/move_base_simple/goal` (geometry_msgs/PoseStamped): 最终目标（由move_base处理）
- Costmap相关话题（由move_base提供）

### 发布
- `~WaypointSnapPlanner/global_plan` (nav_msgs/Path): 生成的全局路径（用于可视化）

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

## 故障排除

### 问题：规划失败
**可能原因**:
- 航点在障碍物中
- 起点到航点之间无可行路径
- costmap未正确配置

**解决方法**:
- 检查RViz中的costmap可视化
- 确保航点在自由空间中
- 增加costmap的inflation_radius

### 问题：轨迹不够平滑
**解决方法**:
- 减小`downsample_resolution`（增加航点密度）
- 增加`polynomial_order`到4
- 检查A*路径是否合理

### 问题：编译错误
**常见问题**:
- 缺少依赖：安装所有ROS导航相关包
- Eigen版本：确保Eigen3已安装
- PCL版本：确保PCL 1.8+已安装

## 性能优化建议

1. **降采样分辨率**: 根据环境复杂度调整
   - 简单环境：0.5-1.0米
   - 复杂环境：0.2-0.5米

2. **多项式阶数**: 
   - 地面机器人：order=2或3
   - 飞行器：order=3或4

3. **Costmap分辨率**: 
   - 与A*搜索效率直接相关
   - 推荐：0.05米

## 引用

如果使用本项目，请引用：

```
Mellinger, D., & Kumar, V. (2011).
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA).
```

原始Minimum Snap实现：
```
Zhang Zhimeng
https://blog.csdn.net/u011341856/article/details/121861930
```

## 许可证

BSD License

## 作者

- 原始实现：Zhang Zhimeng
- Move Base集成与航点支持：基于原项目修改

## 更新日志

### v1.0.0 (2024)
- ✅ 删除3D规划部分
- ✅ 实现move_base全局规划器插件
- ✅ 添加RViz航点选取工具
- ✅ 支持多段A*+Minimum Snap规划
- ✅ 从costmap获取障碍物信息
- ✅ 路径降采样功能
- ✅ 完整的配置文件和文档

## 贡献

欢迎提交Issue和Pull Request！

## 相关资源

- [ROS Navigation Stack](http://wiki.ros.org/navigation)
- [move_base](http://wiki.ros.org/move_base)
- [Writing a Global Path Planner Plugin](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS)

