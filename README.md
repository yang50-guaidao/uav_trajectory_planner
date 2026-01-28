# UAV Trajectory Planner

基于 ROS Noetic 的无人机轨迹规划系统，实现了从路径搜索到轨迹优化的完整流程。

## 功能特性

- **Kinodynamic A\*** - 考虑动力学约束的前端路径搜索
- **安全走廊生成** - 基于 decomp_util 的凸多面体走廊
- **B-spline 轨迹优化** - 使用 OSQP 求解带硬约束的 QP 问题
- **RViz 可视化** - 实时显示地图、路径、走廊和优化轨迹

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    PlannerManager                        │
├─────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌─────────────────┐  ┌────────────────┐ │
│  │ GridMap  │→ │ KinodynamicAstar │→ │CorridorGenerator│ │
│  └──────────┘  └─────────────────┘  └────────────────┘ │
│                                              ↓          │
│                                     ┌────────────────┐  │
│                                     │BsplineOptimizer│  │
│                                     └────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

## 环境要求

- Ubuntu 20.04
- ROS Noetic
- Eigen3
- PCL
- OSQP

## 依赖项

在 `src/` 目录下克隆以下依赖：

```bash
# catkin_simple
git clone https://github.com/catkin/catkin_simple.git

# decomp_util (安全走廊生成)
git clone https://github.com/sikang/DecompUtil.git decomp_util

# decomp_ros_utils (RViz 可视化插件)
git clone https://github.com/sikang/DecompROS.git decomp_ros_utils

# param_env (地图生成器)
git clone https://github.com/HKUST-Aerial-Robotics/param_env.git map_generator
```

安装 osqp-eigen（在工作空间根目录）：

```bash
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../install
make -j4 && make install
cd ../..
```

## 编译

```bash
# 禁用 decomp_util 测试（可选，加速编译）
# 在 src/decomp_util/CMakeLists.txt 开头添加: option(BUILD_TESTING "Build tests" OFF)

# 编译
catkin_make_isolated -j4

# 或者使用 catkin build
catkin build
```

## 使用方法

```bash
# 加载环境
source devel_isolated/setup.bash

# 启动规划器
roslaunch trajectory_planner test_path_search.launch
```

在 RViz 中使用 **2D Nav Goal** 工具点击目标位置，即可看到规划结果。

## 参数配置

配置文件位于 `src/trajectory_planner/config/planner_params.yaml`：

```yaml
# 地图参数
grid_map:
  resolution: 0.1        # 地图分辨率 (m)
  inflate_radius: 0.15   # 障碍物膨胀半径 (m)

# Kinodynamic A* 参数
kinodynamic_astar:
  max_vel: 3.0           # 最大速度 (m/s)
  max_acc: 3.0           # 最大加速度 (m/s²)
  max_iterations: 100000 # 最大迭代次数

# 走廊参数
corridor:
  robot_radius: 0.15     # 机器人半径 (m)
  local_bbox: [1.5, 1.5, 0.8]  # 局部搜索范围 (m)

# B-spline 优化参数
bspline_optimizer:
  lambda_smooth: 10.0    # 平滑度权重
  max_vel: 3.0           # 速度约束 (m/s)
  max_acc: 6.0           # 加速度约束 (m/s²)

# 规划器参数
planner:
  start_pos: [-12.0, -12.0, 1.0]  # 起始位置
```

## 可视化说明

| 颜色 | 含义 |
|------|------|
| 白色点云 | 障碍物地图 |
| 绿色线条 | Kinodynamic A* 搜索路径 |
| 蓝色多面体 | 安全走廊 (Mesh) |
| 红色线框 | 安全走廊边界 (Bound) |
| 紫色椭球 | 膨胀椭球 |
| 红色曲线 | B-spline 优化轨迹 |
| 蓝色球 | B-spline 控制点 |

## 文件结构

```
trajectory_planner/
├── include/trajectory_planner/
│   ├── bspline/
│   │   ├── bspline_optimizer.hpp    # OSQP 优化器
│   │   └── uniform_bspline.hpp      # B-spline 表示
│   ├── corridor/
│   │   └── corridor_generator.hpp   # 走廊生成
│   ├── path_searching/
│   │   └── kinodynamic_astar.hpp    # 前端搜索
│   ├── plan_env/
│   │   └── grid_map.hpp             # 地图处理
│   └── plan_manage/
│       └── planner_manager.hpp      # 规划管理
├── src/
│   ├── bspline/
│   ├── corridor/
│   ├── path_searching/
│   ├── plan_env/
│   ├── plan_manage/
│   └── planner_node.cpp             # ROS 节点入口
├── config/
│   └── planner_params.yaml          # 参数配置
├── launch/
│   └── test_path_search.launch      # 启动文件
└── rviz/
    └── path_search.rviz             # RViz 配置
```

## 预留接口

轨迹采样接口（用于控制器集成）：

```cpp
// 获取优化后的轨迹
UniformBspline traj = planner_manager.getTrajectory();

// 按指定频率采样
std::vector<TrajectoryPoint> samples = traj.sampleTrajectory(100.0);  // 100 Hz

// TrajectoryPoint 包含:
// - position (Eigen::Vector3d)
// - velocity (Eigen::Vector3d)
// - acceleration (Eigen::Vector3d)
// - time (double)
```

## 参考

- [DecompUtil](https://github.com/sikang/DecompUtil) - 安全走廊生成
- [OSQP](https://osqp.org/) - QP 求解器
- [osqp-eigen](https://github.com/robotology/osqp-eigen) - OSQP 的 Eigen 接口

## License

MIT License
