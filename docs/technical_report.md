# 无人机轨迹规划与控制系统技术报告

## 一、项目概述

本项目实现了一个完整的无人机轨迹规划与控制系统，包括：
- 前端路径搜索（Kinodynamic A*）
- 安全飞行走廊生成（decomp_util）
- 后端轨迹优化（B-spline + OSQP）
- 轨迹执行与控制（SO3 Controller）

开发环境：Ubuntu 20.04 + ROS Noetic

---

## 二、技术路线

### 2.1 整体架构

```
用户输入目标点 (2D Nav Goal)
        ↓
┌───────────────────────────────────────────────────────┐
│              Trajectory Planner Node                   │
│  ┌─────────┐   ┌──────────┐   ┌───────────────────┐  │
│  │ GridMap │ → │ Kino A*  │ → │ CorridorGenerator │  │
│  │ (地图)  │   │ (路径)   │   │   (安全走廊)      │  │
│  └─────────┘   └──────────┘   └───────────────────┘  │
│                                        ↓              │
│                              ┌───────────────────┐    │
│                              │ BsplineOptimizer  │    │
│                              │   (轨迹优化)      │    │
│                              └───────────────────┘    │
└───────────────────────────────────────────────────────┘
        ↓ BsplineTrajectory.msg
┌───────────────────────────────────────────────────────┐
│                  Traj Server Node                      │
│         (100Hz 轨迹采样，发布 PositionCommand)         │
└───────────────────────────────────────────────────────┘
        ↓ PositionCommand
┌───────────────────────────────────────────────────────┐
│              kr_mav_control (SO3 Controller)           │
│  ┌──────────────┐   ┌──────────────────────────────┐  │
│  │ SO3 Control  │ → │ Quadrotor Simulator (仿真器) │  │
│  └──────────────┘   └──────────────────────────────┘  │
└───────────────────────────────────────────────────────┘
```

### 2.2 各模块技术选型

| 模块 | 技术方案 | 选型理由 |
|------|----------|----------|
| 前端搜索 | Kinodynamic A* | 考虑动力学约束，生成可执行路径 |
| 安全走廊 | decomp_util | 成熟开源库，生成凸多面体约束 |
| 轨迹表示 | 均匀三次 B-spline | 连续性好，控制点数量可控 |
| 优化求解 | OSQP | 支持硬约束的 QP 求解器 |
| 控制器 | kr_mav_control SO3 | Kumar Robotics 成熟方案 |
| 仿真器 | kr_quadrotor_simulator | 轻量级，无需 Gazebo |

---

## 三、核心算法实现

### 3.1 Kinodynamic A* 路径搜索

**状态空间：** 6 维 (位置 + 速度)
```
State = [x, y, z, vx, vy, vz]
```

**运动基元：** 27 种离散加速度组合
```cpp
// 加速度取值: {-max_acc, 0, max_acc}
for ax in [-a, 0, a]:
    for ay in [-a, 0, a]:
        for az in [-a, 0, a]:
            primitives.add([ax, ay, az])
```

**启发函数：** 考虑时间的欧氏距离
```cpp
h = ||goal_pos - current_pos|| / max_vel * lambda_heuristic
```

**关键调整：**
- 增大 `max_iterations` 至 100000 以处理复杂环境
- 增大 `lambda_heuristic` 至 5.0 加速收敛
- 使用较粗的速度离散化减少状态空间

### 3.2 安全走廊生成 (decomp_util)

**算法流程：**
1. 对每个路径段，以中点为中心膨胀椭球
2. 椭球碰到障碍物时记录法向量
3. 用法向量生成半空间约束（切平面）
4. 半空间交集 + local_bbox = 凸多面体走廊

**核心调用：**
```cpp
EllipsoidDecomp3D decomp;
decomp.set_obs(obstacle_points);
decomp.set_local_bbox(Vec3f(1.5, 1.5, 0.8));  // 搜索范围
decomp.dilate(path);

// 获取结果
auto polyhedrons = decomp.get_polyhedrons();  // 凸多面体
auto constraints = decomp.get_constraints();   // Ax <= b 形式
```

**关键调整：**
- 不设置 global_bbox（与官方示例一致）
- `local_bbox` 设为 [1.5, 1.5, 0.8] 适应室内环境

### 3.3 B-spline 轨迹优化

**优化问题形式：**
```
min   J = λ_smooth * ||D²Q||²    (最小化加加速度)
s.t.  Q[0:3] = 边界约束           (起点位置/速度)
      Q[n-3:n] = 边界约束         (终点位置/速度)
      A_corridor * Q <= b_corridor  (走廊约束)
      |v| <= v_max                 (速度约束)
      |a| <= a_max                 (加速度约束)
```

**B-spline 性质利用：**
- 三次 B-spline：速度 = 一阶导数，加速度 = 二阶导数
- 凸包性质：控制点在走廊内 ⇒ 曲线在走廊内

**OSQP 求解器配置：**
```cpp
OsqpEigen::Solver solver;
solver.settings()->setVerbosity(false);
solver.settings()->setWarmStart(true);
solver.data()->setNumberOfVariables(3 * num_control_points);
solver.data()->setNumberOfConstraints(num_constraints);
```

### 3.4 轨迹执行 (Traj Server)

**功能：** 将 B-spline 轨迹转换为 100Hz 的位置指令

**工作流程：**
1. 订阅 `BsplineTrajectory` 消息
2. 重建 B-spline 控制点
3. 定时器 100Hz 采样当前时刻的位置/速度/加速度
4. 发布 `PositionCommand` 给 SO3 控制器

**Mux 切换机制：**
- 起飞阶段：mux 选择 tracker_cmd（来自 mav_manager）
- 收到轨迹：mux 自动切换到 traj_server 输出
```cpp
void switchMux(bool to_traj_server) {
    topic_tools::MuxSelect srv;
    srv.request.topic = to_traj_server ? 
        "traj_server/position_cmd" : "tracker_cmd";
    mux_client_.call(srv);
}
```

---

## 四、ROS 节点设计

### 4.1 节点通信图

```
                    /move_base_simple/goal
                           ↓
┌──────────────────────────────────────────────────────────────┐
│                   trajectory_planner                          │
│  订阅:                                                        │
│    ~planning/goal     ← 目标点                                │
│    ~odom              ← 无人机当前位置 (用作起点)              │
│    /structure_map/global_cloud  ← 障碍物点云                  │
│  发布:                                                        │
│    ~planning/path          → 搜索路径 (Marker)                │
│    ~planning/trajectory    → 优化轨迹 (Marker)                │
│    ~planning/corridors     → 安全走廊 (PolyhedronArray)       │
│    ~planning/ellipsoids    → 椭球 (EllipsoidArray)            │
│    ~bspline_traj           → 轨迹数据 (BsplineTrajectory)     │
└──────────────────────────────────────────────────────────────┘
                           ↓ /trajectory_planner/bspline_traj
┌──────────────────────────────────────────────────────────────┐
│                      traj_server                              │
│  订阅:                                                        │
│    ~trajectory        ← B-spline 轨迹                         │
│    ~odom              ← 无人机里程计                          │
│  发布:                                                        │
│    ~position_cmd      → 位置指令 (PositionCommand, 100Hz)     │
│  服务调用:                                                    │
│    /quadrotor/cmd_mux/select  → 切换控制源                    │
└──────────────────────────────────────────────────────────────┘
                           ↓ (通过 mux)
┌──────────────────────────────────────────────────────────────┐
│                  /quadrotor/position_cmd                      │
│                         ↓                                     │
│  ┌─────────────┐   ┌──────────┐   ┌───────────────────────┐  │
│  │ SO3Control  │ → │ so3_cmd  │ → │ quadrotor_simulator   │  │
│  └─────────────┘   └──────────┘   └───────────────────────┘  │
│                                            ↓                  │
│                                    /quadrotor/odom            │
└──────────────────────────────────────────────────────────────┘
```

### 4.2 消息定义

**BsplineTrajectory.msg：**
```
Header header
float64[] knots      # 控制点 (flattened: x0,y0,z0,x1,y1,z1,...)
float64 dt           # 时间间隔
float64 duration     # 总时长
float64 start_time   # 开始执行时间
```

### 4.3 Launch 文件结构

**full_simulation.launch 启动顺序：**
1. structure_map (param_env) - 障碍物地图生成
2. quadrotor_simulator_so3 - 四旋翼仿真器
3. mesh_visualization - 无人机模型可视化
4. nodelet_manager_control - Nodelet 管理器
5. so3_control - SO3 控制器
6. trackers_manager - 轨迹跟踪管理（起飞用）
7. mav_services - MAV 服务（Motors ON, Take Off）
8. cmd_mux - 指令多路复用器
9. trajectory_planner - 轨迹规划器
10. traj_server - 轨迹执行服务器
11. rqt_mav_manager - GUI 控制面板

---

## 五、关键问题与解决方案

### 5.1 编译问题

| 问题 | 解决方案 |
|------|----------|
| decomp_util 测试编译卡住 | CMakeLists.txt 添加 `option(BUILD_TESTING OFF)` |
| 找不到 catkin_simple | 克隆 catkin_simple 到 src/ |
| osqp-eigen 链接失败 | 使用 HINTS 指定安装路径 |

### 5.2 算法调优

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| A* 只能规划短路径 | 迭代次数不足 | max_iterations: 100000 |
| 搜索速度慢 | 启发函数权重低 | lambda_heuristic: 5.0 |
| 初始点被障碍物覆盖 | 膨胀半径过大 | inflate_radius: 0.15 |
| 走廊太大 | local_bbox 过大 | local_bbox: [1.5, 1.5, 0.8] |

### 5.3 控制器集成

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| Motors ON 无反应 | mav_services 依赖 trackers_manager | 恢复 trackers_manager 节点 |
| 起飞后飞到无限高 | mux 默认选择了 traj_server | 启动时切换到 tracker_cmd |
| 指令冲突 | 多个节点发布 position_cmd | 使用 topic_tools/mux 切换 |
| 轨迹不从当前位置开始 | 使用固定起点 | 订阅 odom 作为起点 |

---

## 六、系统参数配置

### 6.1 规划器参数 (planner_params.yaml)

```yaml
grid_map:
  resolution: 0.1        # 地图分辨率
  inflate_radius: 0.15   # 障碍物膨胀

kinodynamic_astar:
  max_vel: 3.0           # 最大速度
  max_acc: 3.0           # 最大加速度
  max_iterations: 100000 # 最大迭代
  lambda_heuristic: 5.0  # 启发权重
  goal_tolerance: 1.0    # 目标容差

corridor:
  robot_radius: 0.15     # 机器人半径
  local_bbox: [1.5, 1.5, 0.8]  # 走廊范围

bspline_optimizer:
  lambda_smooth: 10.0    # 平滑权重
  max_vel: 3.0           # 速度约束
  max_acc: 6.0           # 加速度约束

planner:
  use_odom_as_start: true  # 使用当前位置作为起点
```

### 6.2 控制器参数 (gains.yaml)

```yaml
gains:
  pos: {x: 7.4, y: 7.4, z: 10.4}   # 位置增益
  vel: {x: 4.8, y: 4.8, z: 6.0}    # 速度增益
```

---

## 七、测试结果

### 7.1 功能验证

- [x] Kinodynamic A* 路径搜索
- [x] decomp_util 安全走廊生成
- [x] B-spline 轨迹优化
- [x] 走廊约束满足
- [x] 速度/加速度约束满足
- [x] SO3 控制器轨迹跟踪
- [x] 从当前位置重新规划
- [x] RViz 可视化

### 7.2 性能指标

| 指标 | 数值 |
|------|------|
| A* 搜索时间 | ~50-200ms |
| 走廊生成时间 | ~10-50ms |
| 轨迹优化时间 | ~5-20ms |
| 总规划时间 | ~100-300ms |
| 指令发布频率 | 100Hz |

---

## 八、后续改进方向

1. **动态避障** - 加入局部重规划
2. **时间最优** - 优化轨迹时间分配
3. **实机部署** - 接入真实飞控
4. **多机协同** - 扩展到多无人机场景

---

## 九、代码仓库

GitHub: https://github.com/yang50-guaidao/uav_trajectory_planner

## 十、参考文献

1. S. Liu, et al. "Planning Dynamically Feasible Trajectories for Quadrotors Using Safe Flight Corridors in 3-D Complex Environments." IEEE RAL, 2017.
2. W. Zhou, et al. "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight." IEEE RAL, 2019.
3. Kumar Robotics. kr_mav_control. https://github.com/KumarRobotics/kr_mav_control
