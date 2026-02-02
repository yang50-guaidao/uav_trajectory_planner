# UAV Trajectory Planner

基于 ROS Noetic 的无人机轨迹规划与控制系统。

## 环境

- Ubuntu 20.04
- ROS Noetic
- Eigen3, PCL, OSQP

## 依赖安装

```bash
cd ~/catkin_ws/src

# 必需依赖
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/sikang/DecompUtil.git decomp_util
git clone https://github.com/sikang/DecompROS.git decomp_ros_utils
git clone https://github.com/HKUST-Aerial-Robotics/param_env.git map_generator
git clone https://github.com/KumarRobotics/kr_mav_control.git

# osqp-eigen (在工作空间根目录)
cd ..
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../install
make -j4 && make install
```

## 编译

```bash
# 可选：禁用 decomp_util 测试加速编译
# 在 src/decomp_util/CMakeLists.txt 添加: option(BUILD_TESTING "Build tests" OFF)

catkin_make_isolated -j4
source devel_isolated/setup.bash
```

## 运行

**仅规划（无控制器）：**
```bash
roslaunch trajectory_planner test_path_search.launch
```
<img width="1886" height="1188" alt="7df468bf253147b639134018ecefa907" src="https://github.com/user-attachments/assets/eefc0699-b448-4141-8a49-3eb854025f1a" />


**完整仿真（规划 + SO3控制器）：**
```bash
# 终端1: 启动仿真
roslaunch trajectory_planner full_simulation.launch

# 终端2: 启动 RViz
rviz
# 设置 Fixed Frame: world
# 添加: PointCloud2 (/structure_map/global_cloud)
# 添加: Marker (/trajectory_planner/planning/trajectory)
# 添加: Odometry (/quadrotor/odom)
```
![Video-Project](https://github.com/user-attachments/assets/0385b676-1759-4c1d-8008-330eb1df4a7b)


**操作步骤：**
1. rqt_mav_manager 中点击 Motors ON → Take Off
2. RViz 中用 2D Nav Goal 设置目标
3. 无人机自动跟踪轨迹

## 主要参数

配置文件: `config/planner_params.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| max_vel | 3.0 | 最大速度 (m/s) |
| max_acc | 3.0/6.0 | 最大加速度 (m/s²) |
| local_bbox | [1.5,1.5,0.8] | 走廊搜索范围 (m) |
| inflate_radius | 0.15 | 障碍物膨胀 (m) |

## License

MIT
