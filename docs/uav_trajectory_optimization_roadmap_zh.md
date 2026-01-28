# 无人机杂乱环境轨迹优化技术路线

## 面向CURSOR编程的详细实现指南

构建一个能在杂乱空间中导航的实时四旋翼轨迹规划器，需要集成五个核心系统：环境建图、路径搜索、轨迹优化、安全走廊生成和底层控制。本技术路线为基于ROS1 Noetic的系统提供可直接实现的规范，采用B样条轨迹表示，集成指定的kr_mav_control控制栈和map_generator地图生成器。

整体架构遵循经过验证的Fast-Planner/EGO-Planner范式：前端运动学搜索生成初始路径，后端B样条优化器对其进行平滑性、避障和动力学可行性优化。采用EGO-Planner的无ESDF梯度计算方法，规划时间可达**5ms以内**；而GCOPTER的MINCO轨迹表示则为对时间要求不那么严格的应用提供更优的轨迹质量。

---

## 一、系统架构概览

### 1.1 五模块互联架构

规划管线通过模块化架构处理传感器数据，每个组件都有明确定义的接口。map_generator在`/map_generator/global_cloud`话题上发布障碍物点云（`sensor_msgs/PointCloud2`类型），直接输入到ESDF计算层。规划器的输出通过位置指令连接到kr_mav_control，由SO3控制器转换为姿态和推力。

```
┌─────────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  map_generator  │───►│  plan_env   │───►│path_search  │───►│ bspline_opt │
│  (PointCloud2)  │    │  (ESDF)     │    │(Kinodyn A*) │    │ (L-BFGS)    │
└─────────────────┘    └─────────────┘    └─────────────┘    └──────┬──────┘
                                                                    │
┌─────────────────┐    ┌─────────────┐    ┌─────────────┐          │
│ kr_mav_control  │◄───│ SO3Control  │◄───│ traj_server │◄─────────┘
│   (电机指令)     │    │(PositionCmd)│    │(100Hz采样)  │
└─────────────────┘    └─────────────┘    └─────────────┘
```

### 1.2 模块划分（参照Fast-Planner结构）

| 模块名称 | 功能描述 | 主要依赖 |
|---------|---------|---------|
| `plan_env` | ESDF建图和距离查询 | Voxblox / 自实现Grid Map |
| `path_searching` | 运动学A*和JPS前端搜索 | Eigen, jps3d |
| `bspline_opt` | 基于梯度的B样条轨迹优化 | LBFGS-Lite, Eigen |
| `plan_manage` | 状态机逻辑协调整个管线 | ROS |
| `traj_server` | 以100Hz频率采样轨迹供控制器使用 | kr_mav_msgs |

---

## 二、B样条轨迹优化详细实现

### 2.1 B样条数学基础

均匀三次B样条通过控制点**Q** = {Q₀, Q₁, ..., Qₙ}表示轨迹，其**凸包性质**保证：如果所有控制点都位于凸安全区域内，则整个轨迹段也保持无碰撞。

**三次B样条基矩阵：**
```cpp
// 文件: bspline_opt/include/bspline_base.hpp

#include <Eigen/Dense>

class UniformBspline {
public:
    // 三次B样条基矩阵
    static Eigen::Matrix4d M_cubic;
    
    static void initBasisMatrix() {
        M_cubic << -1,  3, -3,  1,
                    3, -6,  3,  0,
                   -3,  0,  3,  0,
                    1,  4,  1,  0;
        M_cubic /= 6.0;
    }
    
    // 在参数u∈[0,1]处求值，使用局部4个控制点
    static Eigen::Vector3d evaluate(double u, const Eigen::Matrix<double, 3, 4>& ctrl_pts) {
        Eigen::Vector4d U;
        U << u*u*u, u*u, u, 1;
        return ctrl_pts * M_cubic.transpose() * U;
    }
    
    // 一阶导数（速度）
    static Eigen::Vector3d evaluateDerivative(double u, double dt, 
                                               const Eigen::Matrix<double, 3, 4>& ctrl_pts) {
        Eigen::Vector4d dU;
        dU << 3*u*u, 2*u, 1, 0;
        return ctrl_pts * M_cubic.transpose() * dU / dt;
    }
    
    // 二阶导数（加速度）
    static Eigen::Vector3d evaluateSecondDerivative(double u, double dt,
                                                     const Eigen::Matrix<double, 3, 4>& ctrl_pts) {
        Eigen::Vector4d ddU;
        ddU << 6*u, 2, 0, 0;
        return ctrl_pts * M_cubic.transpose() * ddU / (dt * dt);
    }
};

Eigen::Matrix4d UniformBspline::M_cubic = Eigen::Matrix4d::Zero();
```

### 2.2 B样条轨迹类完整实现

```cpp
// 文件: bspline_opt/include/bspline_trajectory.hpp

#pragma once
#include <Eigen/Dense>
#include <vector>

class BsplineTrajectory {
private:
    std::vector<Eigen::Vector3d> control_points_;  // 控制点序列
    double dt_;                                      // 时间间隔
    int degree_;                                     // 样条阶数（默认3）
    
public:
    BsplineTrajectory() : dt_(0.1), degree_(3) {}
    
    // 从路径点初始化控制点
    void initFromPath(const std::vector<Eigen::Vector3d>& path, double total_time) {
        int n = path.size();
        dt_ = total_time / (n - 1);
        
        // 简单初始化：路径点作为控制点（可后续优化）
        control_points_ = path;
        
        // 添加端点约束所需的额外控制点
        // 起点速度为零约束
        control_points_.insert(control_points_.begin(), path[0]);
        // 终点速度为零约束  
        control_points_.push_back(path.back());
    }
    
    // 获取指定时间的位置
    Eigen::Vector3d getPosition(double t) const {
        if (control_points_.size() < 4) return Eigen::Vector3d::Zero();
        
        double total_time = (control_points_.size() - 3) * dt_;
        t = std::max(0.0, std::min(t, total_time - 1e-6));
        
        int seg_idx = static_cast<int>(t / dt_);
        seg_idx = std::min(seg_idx, static_cast<int>(control_points_.size()) - 4);
        
        double u = (t - seg_idx * dt_) / dt_;
        
        Eigen::Matrix<double, 3, 4> local_ctrl;
        for (int i = 0; i < 4; i++) {
            local_ctrl.col(i) = control_points_[seg_idx + i];
        }
        
        return UniformBspline::evaluate(u, local_ctrl);
    }
    
    // 获取指定时间的速度
    Eigen::Vector3d getVelocity(double t) const {
        if (control_points_.size() < 4) return Eigen::Vector3d::Zero();
        
        double total_time = (control_points_.size() - 3) * dt_;
        t = std::max(0.0, std::min(t, total_time - 1e-6));
        
        int seg_idx = static_cast<int>(t / dt_);
        seg_idx = std::min(seg_idx, static_cast<int>(control_points_.size()) - 4);
        
        double u = (t - seg_idx * dt_) / dt_;
        
        Eigen::Matrix<double, 3, 4> local_ctrl;
        for (int i = 0; i < 4; i++) {
            local_ctrl.col(i) = control_points_[seg_idx + i];
        }
        
        return UniformBspline::evaluateDerivative(u, dt_, local_ctrl);
    }
    
    // 获取指定时间的加速度
    Eigen::Vector3d getAcceleration(double t) const {
        if (control_points_.size() < 4) return Eigen::Vector3d::Zero();
        
        double total_time = (control_points_.size() - 3) * dt_;
        t = std::max(0.0, std::min(t, total_time - 1e-6));
        
        int seg_idx = static_cast<int>(t / dt_);
        seg_idx = std::min(seg_idx, static_cast<int>(control_points_.size()) - 4);
        
        double u = (t - seg_idx * dt_) / dt_;
        
        Eigen::Matrix<double, 3, 4> local_ctrl;
        for (int i = 0; i < 4; i++) {
            local_ctrl.col(i) = control_points_[seg_idx + i];
        }
        
        return UniformBspline::evaluateSecondDerivative(u, dt_, local_ctrl);
    }
    
    // 获取总时长
    double getTotalTime() const {
        return (control_points_.size() - 3) * dt_;
    }
    
    // 获取/设置控制点（用于优化）
    std::vector<Eigen::Vector3d>& getControlPoints() { return control_points_; }
    const std::vector<Eigen::Vector3d>& getControlPoints() const { return control_points_; }
    
    double getDt() const { return dt_; }
    void setDt(double dt) { dt_ = dt; }
};
```

### 2.3 优化目标函数实现

```cpp
// 文件: bspline_opt/include/bspline_optimizer.hpp

#pragma once
#include "bspline_trajectory.hpp"
#include "lbfgs.hpp"  // LBFGS-Lite头文件
#include <functional>

class BsplineOptimizer {
public:
    // 优化参数
    struct Parameters {
        double lambda_smooth = 1.0;      // 平滑权重
        double lambda_collision = 10.0;   // 避碰权重
        double lambda_feasibility = 1.0;  // 动力学约束权重
        double max_vel = 3.0;             // 最大速度 m/s
        double max_acc = 6.0;             // 最大加速度 m/s²
        double safe_distance = 0.5;       // 安全距离 m
        int max_iteration = 100;          // 最大迭代次数
    };
    
private:
    Parameters params_;
    BsplineTrajectory* traj_;
    
    // 距离查询函数（由外部提供，如ESDF）
    std::function<double(const Eigen::Vector3d&)> dist_func_;
    std::function<Eigen::Vector3d(const Eigen::Vector3d&)> grad_func_;
    
    // 无碰撞引导路径（EGO-Planner方法）
    std::vector<Eigen::Vector3d> guide_path_;
    
public:
    void setParameters(const Parameters& params) { params_ = params; }
    void setTrajectory(BsplineTrajectory* traj) { traj_ = traj; }
    void setDistanceFunction(std::function<double(const Eigen::Vector3d&)> f) { dist_func_ = f; }
    void setGradientFunction(std::function<Eigen::Vector3d(const Eigen::Vector3d&)> f) { grad_func_ = f; }
    void setGuidePath(const std::vector<Eigen::Vector3d>& path) { guide_path_ = path; }
    
    // ==================== 代价函数计算 ====================
    
    // 平滑代价：最小化Jerk（三阶导数）
    double computeSmoothnessCost(Eigen::VectorXd& gradient) {
        auto& Q = traj_->getControlPoints();
        double dt = traj_->getDt();
        int n = Q.size();
        double cost = 0.0;
        
        gradient.setZero();
        
        // J_smooth = Σ ||Q_{i+1} - 2Q_i + Q_{i-1}||² / dt⁴
        for (int i = 1; i < n - 1; i++) {
            Eigen::Vector3d diff = Q[i+1] - 2.0 * Q[i] + Q[i-1];
            cost += diff.squaredNorm() / (dt * dt * dt * dt);
            
            // 梯度计算
            Eigen::Vector3d grad_i = 2.0 * (-2.0) * diff / (dt * dt * dt * dt);
            gradient.segment<3>(3*i) += grad_i;
            
            if (i > 1) {
                gradient.segment<3>(3*(i-1)) += 2.0 * diff / (dt * dt * dt * dt);
            }
            if (i < n - 2) {
                gradient.segment<3>(3*(i+1)) += 2.0 * diff / (dt * dt * dt * dt);
            }
        }
        
        return cost;
    }
    
    // 碰撞代价（EGO-Planner方法：无需ESDF）
    double computeCollisionCostEGO(Eigen::VectorXd& gradient) {
        auto& Q = traj_->getControlPoints();
        int n = Q.size();
        double cost = 0.0;
        
        gradient.setZero();
        
        // 对每个控制点检查碰撞
        for (int i = 1; i < n - 1; i++) {
            double dist = dist_func_(Q[i]);
            
            if (dist < params_.safe_distance) {
                // 找到引导路径上的最近点
                Eigen::Vector3d p_guide = findNearestGuidePoint(Q[i]);
                
                // 代价：到引导点的距离
                Eigen::Vector3d diff = Q[i] - p_guide;
                cost += diff.squaredNorm();
                
                // 梯度
                gradient.segment<3>(3*i) += 2.0 * diff;
            }
        }
        
        return cost;
    }
    
    // 碰撞代价（传统ESDF方法）
    double computeCollisionCostESDF(Eigen::VectorXd& gradient) {
        auto& Q = traj_->getControlPoints();
        int n = Q.size();
        double cost = 0.0;
        
        gradient.setZero();
        
        for (int i = 1; i < n - 1; i++) {
            double dist = dist_func_(Q[i]);
            
            if (dist < params_.safe_distance) {
                // 惩罚函数：(safe_dist - dist)²
                double pen = params_.safe_distance - dist;
                cost += pen * pen;
                
                // ESDF梯度
                Eigen::Vector3d esdf_grad = grad_func_(Q[i]);
                gradient.segment<3>(3*i) += -2.0 * pen * esdf_grad;
            }
        }
        
        return cost;
    }
    
    // 可行性代价：速度和加速度约束
    double computeFeasibilityCost(Eigen::VectorXd& gradient) {
        auto& Q = traj_->getControlPoints();
        double dt = traj_->getDt();
        int n = Q.size();
        double cost = 0.0;
        
        gradient.setZero();
        
        // 速度约束：v_i = (Q_{i+1} - Q_i) / dt
        for (int i = 0; i < n - 1; i++) {
            Eigen::Vector3d vel = (Q[i+1] - Q[i]) / dt;
            double vel_norm = vel.norm();
            
            if (vel_norm > params_.max_vel) {
                double excess = vel_norm - params_.max_vel;
                cost += excess * excess;
                
                // 梯度
                Eigen::Vector3d grad = 2.0 * excess * vel / (vel_norm * dt);
                gradient.segment<3>(3*i) -= grad;
                gradient.segment<3>(3*(i+1)) += grad;
            }
        }
        
        // 加速度约束：a_i = (Q_{i+1} - 2Q_i + Q_{i-1}) / dt²
        for (int i = 1; i < n - 1; i++) {
            Eigen::Vector3d acc = (Q[i+1] - 2.0*Q[i] + Q[i-1]) / (dt * dt);
            double acc_norm = acc.norm();
            
            if (acc_norm > params_.max_acc) {
                double excess = acc_norm - params_.max_acc;
                cost += excess * excess;
                
                // 梯度
                Eigen::Vector3d grad = 2.0 * excess * acc / (acc_norm * dt * dt);
                gradient.segment<3>(3*(i-1)) += grad;
                gradient.segment<3>(3*i) -= 2.0 * grad;
                gradient.segment<3>(3*(i+1)) += grad;
            }
        }
        
        return cost;
    }
    
    // ==================== 优化主函数 ====================
    
    bool optimize() {
        auto& Q = traj_->getControlPoints();
        int n = Q.size();
        int dim = 3 * (n - 2);  // 固定首尾控制点，只优化中间点
        
        // 将控制点展平为一维数组
        Eigen::VectorXd x(dim);
        for (int i = 1; i < n - 1; i++) {
            x.segment<3>(3*(i-1)) = Q[i];
        }
        
        // L-BFGS优化
        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);
        param.epsilon = 1e-6;
        param.max_iterations = params_.max_iteration;
        
        double fx;
        int ret = lbfgs(dim, x.data(), &fx, 
                        &BsplineOptimizer::costFunctionWrapper, 
                        nullptr, this, &param);
        
        // 更新控制点
        for (int i = 1; i < n - 1; i++) {
            Q[i] = x.segment<3>(3*(i-1));
        }
        
        return (ret >= 0 || ret == LBFGS_STOP);
    }
    
private:
    // L-BFGS回调函数
    static lbfgsfloatval_t costFunctionWrapper(void* instance, 
                                                const lbfgsfloatval_t* x,
                                                lbfgsfloatval_t* g, 
                                                const int n,
                                                const lbfgsfloatval_t step) {
        return reinterpret_cast<BsplineOptimizer*>(instance)->computeTotalCost(x, g, n);
    }
    
    double computeTotalCost(const double* x, double* g, int n) {
        // 临时更新控制点
        auto& Q = traj_->getControlPoints();
        for (int i = 1; i < (int)Q.size() - 1; i++) {
            Q[i] = Eigen::Vector3d(x[3*(i-1)], x[3*(i-1)+1], x[3*(i-1)+2]);
        }
        
        Eigen::VectorXd grad_smooth(n), grad_collision(n), grad_feasibility(n);
        grad_smooth.setZero();
        grad_collision.setZero();
        grad_feasibility.setZero();
        
        double cost_smooth = computeSmoothnessCost(grad_smooth);
        double cost_collision = computeCollisionCostEGO(grad_collision);
        double cost_feasibility = computeFeasibilityCost(grad_feasibility);
        
        double total_cost = params_.lambda_smooth * cost_smooth +
                           params_.lambda_collision * cost_collision +
                           params_.lambda_feasibility * cost_feasibility;
        
        // 合并梯度
        Eigen::Map<Eigen::VectorXd> grad(g, n);
        grad = params_.lambda_smooth * grad_smooth +
               params_.lambda_collision * grad_collision +
               params_.lambda_feasibility * grad_feasibility;
        
        return total_cost;
    }
    
    Eigen::Vector3d findNearestGuidePoint(const Eigen::Vector3d& pt) {
        double min_dist = std::numeric_limits<double>::max();
        Eigen::Vector3d nearest = pt;
        
        for (const auto& gp : guide_path_) {
            double d = (pt - gp).norm();
            if (d < min_dist) {
                min_dist = d;
                nearest = gp;
            }
        }
        
        return nearest;
    }
};
```

### 2.4 典型优化参数配置

```yaml
# config/bspline_optimizer.yaml

bspline_optimizer:
  # 代价函数权重
  lambda_smooth: 1.0        # 平滑权重
  lambda_collision: 10.0    # 避碰权重（可调大到20以提高安全性）
  lambda_feasibility: 1.0   # 动力学约束权重
  
  # 动力学约束
  max_vel: 3.0              # 最大速度 m/s
  max_acc: 6.0              # 最大加速度 m/s²
  
  # 安全参数
  safe_distance: 0.5        # 安全距离 m（应大于机器人半径）
  
  # 优化器参数
  max_iteration: 100        # 最大迭代次数
  ctrl_pt_dist: 0.4         # 控制点间距 m
  
  # 时间分配
  init_time_factor: 1.2     # 初始时间分配因子（路径长度/最大速度 × 因子）
```

---

## 三、MINCO轨迹表示（高级可选）

MINCO（最小控制）来自浙大FAST实验室的GCOPTER，通过**稀疏参数**表示轨迹，实现更优的轨迹质量。

### 3.1 MINCO类结构

```cpp
// 文件: minco_opt/include/minco.hpp
// 参考: https://github.com/ZJU-FAST-Lab/GCOPTER

template <int D>  // D = 维度（位置为3）
class MINCO_S4 {
public:
    // 稀疏参数
    Eigen::Matrix<double, D, Eigen::Dynamic> innerPoints;  // m个路点 × D维
    Eigen::VectorXd durations;                              // m个段时长
    
    // 多项式系数（由innerPoints和durations确定）
    std::vector<Eigen::Matrix<double, D, 8>> coefficients;  // 7阶多项式
    
    void setConditions(const Eigen::Matrix<double, D, 1>& start_pos,
                       const Eigen::Matrix<double, D, 1>& start_vel,
                       const Eigen::Matrix<double, D, 1>& end_pos,
                       const Eigen::Matrix<double, D, 1>& end_vel,
                       int num_pieces) {
        // 设置边界条件
        // ...
    }
    
    void generateTraj() {
        // 根据innerPoints和durations计算多项式系数
        // 使用线性映射 C = M(q, T)
        // ...
    }
    
    void getGrad(Eigen::Matrix<double, D, Eigen::Dynamic>& grad_inner_pts,
                 Eigen::VectorXd& grad_durations) {
        // 计算解析梯度
        // ...
    }
};
```

### 3.2 选择建议

| 方法 | 优点 | 缺点 | 适用场景 |
|-----|------|------|---------|
| **B样条** | 实现简单、局部控制性好 | 时间分配固定 | 实时重规划(<5ms) |
| **MINCO** | 时空联合优化、轨迹质量高 | 实现复杂 | 高质量轨迹、走廊约束 |

**推荐：** 先用B样条实现基础功能，后期可升级到MINCO。

---

## 四、安全飞行走廊生成

### 4.1 IRIS算法实现框架

```cpp
// 文件: corridor_gen/include/iris_corridor.hpp

#include <Eigen/Dense>
#include <vector>

struct Polyhedron {
    Eigen::MatrixXd A;  // 半平面法向量矩阵 (m × 3)
    Eigen::VectorXd b;  // 半平面偏移向量 (m × 1)
    // 表示: A * x <= b
};

class IRISCorridorGenerator {
public:
    struct Parameters {
        int max_iterations = 10;
        double min_volume_change = 0.01;  // 收敛阈值
        double initial_ellipsoid_radius = 0.5;
    };
    
private:
    Parameters params_;
    std::vector<Eigen::Vector3d> obstacles_;  // 障碍物点集
    
public:
    void setObstacles(const std::vector<Eigen::Vector3d>& obs) {
        obstacles_ = obs;
    }
    
    // 生成单个走廊
    Polyhedron generateCorridor(const Eigen::Vector3d& seed_point) {
        Polyhedron poly;
        Eigen::Matrix3d ellipsoid = Eigen::Matrix3d::Identity() * 
                                     params_.initial_ellipsoid_radius;
        Eigen::Vector3d center = seed_point;
        
        for (int iter = 0; iter < params_.max_iterations; iter++) {
            // Step 1: 生成分离超平面
            generateSeparatingHyperplanes(center, ellipsoid, poly);
            
            // Step 2: 找最大体积内切椭球（简化版：可用SOCP替代SDP）
            double prev_volume = ellipsoid.determinant();
            findMaxInscribedEllipsoid(poly, center, ellipsoid);
            
            // 检查收敛
            double volume_change = std::abs(ellipsoid.determinant() - prev_volume) / prev_volume;
            if (volume_change < params_.min_volume_change) break;
        }
        
        return poly;
    }
    
    // 沿路径生成走廊序列
    std::vector<Polyhedron> generateCorridorSequence(
            const std::vector<Eigen::Vector3d>& path) {
        std::vector<Polyhedron> corridors;
        
        for (const auto& pt : path) {
            Polyhedron poly = generateCorridor(pt);
            corridors.push_back(poly);
        }
        
        return corridors;
    }
    
private:
    void generateSeparatingHyperplanes(const Eigen::Vector3d& center,
                                        const Eigen::Matrix3d& ellipsoid,
                                        Polyhedron& poly) {
        // 对每个障碍物点，找到分离超平面
        std::vector<Eigen::Vector3d> normals;
        std::vector<double> offsets;
        
        for (const auto& obs : obstacles_) {
            Eigen::Vector3d diff = obs - center;
            
            // 超平面法向量：从中心指向障碍物
            Eigen::Vector3d normal = ellipsoid.inverse() * diff;
            normal.normalize();
            
            // 超平面位置：在椭球表面与障碍物中点
            double d = diff.dot(normal);
            double offset = center.dot(normal) + d * 0.5;
            
            normals.push_back(normal);
            offsets.push_back(offset);
        }
        
        poly.A.resize(normals.size(), 3);
        poly.b.resize(normals.size());
        for (size_t i = 0; i < normals.size(); i++) {
            poly.A.row(i) = normals[i].transpose();
            poly.b(i) = offsets[i];
        }
    }
    
    void findMaxInscribedEllipsoid(const Polyhedron& poly,
                                    Eigen::Vector3d& center,
                                    Eigen::Matrix3d& ellipsoid) {
        // 简化实现：使用迭代收缩方法
        // 完整实现应使用MOSEK等SOCP求解器
        // 参考: https://github.com/RobotLocomotion/drake (IRIS implementation)
        
        // 这里提供简化版本
        for (int i = 0; i < poly.A.rows(); i++) {
            Eigen::Vector3d normal = poly.A.row(i).transpose();
            double dist_to_plane = poly.b(i) - center.dot(normal);
            
            if (dist_to_plane < 0) {
                // 中心在约束外，移动中心
                center += normal * (-dist_to_plane + 0.1);
            }
        }
    }
};
```

### 4.2 凸分解简化方法（适用于凸障碍物）

```cpp
// 文件: corridor_gen/include/convex_decomposition.hpp

class SimpleCorridorGenerator {
public:
    // 基于膨胀的简单走廊生成
    Polyhedron generateAxisAlignedBox(const Eigen::Vector3d& center,
                                       double safe_radius,
                                       const std::function<double(const Eigen::Vector3d&)>& dist_func) {
        Polyhedron poly;
        
        // 6个面的法向量（轴对齐盒子）
        poly.A.resize(6, 3);
        poly.b.resize(6);
        
        poly.A << 1, 0, 0,
                 -1, 0, 0,
                  0, 1, 0,
                  0,-1, 0,
                  0, 0, 1,
                  0, 0,-1;
        
        // 沿每个方向膨胀直到遇到障碍物
        std::vector<double> extents(6);
        
        for (int i = 0; i < 6; i++) {
            Eigen::Vector3d dir = poly.A.row(i).transpose();
            double extent = 0.1;
            
            // 射线搜索
            while (extent < 5.0) {  // 最大膨胀距离
                Eigen::Vector3d test_pt = center + dir * extent;
                if (dist_func(test_pt) < safe_radius) break;
                extent += 0.1;
            }
            
            extents[i] = extent - 0.1;
        }
        
        // 设置偏移
        poly.b << center.x() + extents[0],
                 -center.x() + extents[1],
                  center.y() + extents[2],
                 -center.y() + extents[3],
                  center.z() + extents[4],
                 -center.z() + extents[5];
        
        return poly;
    }
};
```

### 4.3 走廊约束应用到B样条

```cpp
// 将走廊约束加入优化问题
// 由于B样条凸包性质，约束控制点即可保证轨迹无碰撞

double computeCorridorCost(const std::vector<Polyhedron>& corridors,
                            Eigen::VectorXd& gradient) {
    auto& Q = traj_->getControlPoints();
    int n = Q.size();
    double cost = 0.0;
    
    gradient.setZero();
    
    // 每个控制点应在对应走廊内
    for (int i = 0; i < n; i++) {
        // 找到该控制点对应的走廊
        int corridor_idx = i * corridors.size() / n;
        corridor_idx = std::min(corridor_idx, (int)corridors.size() - 1);
        
        const Polyhedron& poly = corridors[corridor_idx];
        
        // 检查所有半平面约束
        for (int j = 0; j < poly.A.rows(); j++) {
            Eigen::Vector3d normal = poly.A.row(j).transpose();
            double violation = Q[i].dot(normal) - poly.b(j);
            
            if (violation > 0) {
                // 违反约束，添加惩罚
                cost += violation * violation;
                gradient.segment<3>(3*i) += 2.0 * violation * normal;
            }
        }
    }
    
    return cost;
}
```

---

## 五、前端路径搜索

### 5.1 运动学A*实现

```cpp
// 文件: path_searching/include/kinodynamic_astar.hpp

#pragma once
#include <Eigen/Dense>
#include <queue>
#include <unordered_map>
#include <vector>

class KinodynamicAstar {
public:
    struct State {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        double g_cost;
        double f_cost;
        int parent_idx;
        Eigen::Vector3d input;  // 到达该状态的控制输入
        double duration;        // 到达该状态的时间
    };
    
    struct Parameters {
        double max_vel = 3.0;
        double max_acc = 6.0;
        double resolution = 0.5;       // 位置分辨率
        double time_resolution = 0.1;  // 时间分辨率
        double lambda_heuristic = 1.0; // 启发式权重
        int max_iterations = 10000;
    };
    
private:
    Parameters params_;
    std::function<double(const Eigen::Vector3d&)> dist_func_;
    double safe_distance_;
    
    // 离散控制输入
    std::vector<Eigen::Vector3d> control_inputs_;
    std::vector<double> time_durations_;
    
public:
    KinodynamicAstar() {
        // 初始化控制输入集合
        std::vector<double> acc_samples = {-params_.max_acc, 0, params_.max_acc};
        
        for (double ax : acc_samples) {
            for (double ay : acc_samples) {
                for (double az : acc_samples) {
                    control_inputs_.push_back(Eigen::Vector3d(ax, ay, az));
                }
            }
        }
        
        // 时间离散
        time_durations_ = {0.5, 0.1, 0.2};
    }
    
    void setDistanceFunction(std::function<double(const Eigen::Vector3d&)> f) {
        dist_func_ = f;
    }
    
    void setSafeDistance(double d) { safe_distance_ = d; }
    
    // 主搜索函数
    bool search(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                const Eigen::Vector3d& goal_pos, const Eigen::Vector3d& goal_vel,
                std::vector<Eigen::Vector3d>& path) {
        
        // 优先队列（按f_cost排序）
        auto cmp = [](const State& a, const State& b) { return a.f_cost > b.f_cost; };
        std::priority_queue<State, std::vector<State>, decltype(cmp)> open_set(cmp);
        
        // 已访问状态
        std::unordered_map<int, State> closed_set;
        std::vector<State> all_states;
        
        // 初始状态
        State start;
        start.pos = start_pos;
        start.vel = start_vel;
        start.g_cost = 0;
        start.f_cost = computeHeuristic(start_pos, start_vel, goal_pos, goal_vel);
        start.parent_idx = -1;
        
        all_states.push_back(start);
        open_set.push(start);
        
        int iterations = 0;
        while (!open_set.empty() && iterations < params_.max_iterations) {
            iterations++;
            
            State current = open_set.top();
            open_set.pop();
            
            // 检查是否到达目标
            if ((current.pos - goal_pos).norm() < params_.resolution &&
                (current.vel - goal_vel).norm() < params_.max_vel * 0.1) {
                // 回溯路径
                path.clear();
                int idx = all_states.size() - 1;
                while (idx >= 0) {
                    path.push_back(all_states[idx].pos);
                    idx = all_states[idx].parent_idx;
                }
                std::reverse(path.begin(), path.end());
                return true;
            }
            
            // 尝试解析扩展（直接连接到目标）
            if (tryAnalyticExpansion(current, goal_pos, goal_vel, path)) {
                return true;
            }
            
            // 扩展邻居
            for (const auto& u : control_inputs_) {
                for (double tau : time_durations_) {
                    State next;
                    
                    // 双积分器前向传播
                    next.pos = current.pos + current.vel * tau + 0.5 * u * tau * tau;
                    next.vel = current.vel + u * tau;
                    
                    // 速度约束检查
                    if (next.vel.norm() > params_.max_vel) continue;
                    
                    // 碰撞检查
                    if (!checkCollisionFree(current.pos, next.pos)) continue;
                    
                    // 代价计算
                    next.g_cost = current.g_cost + tau + 0.1 * u.squaredNorm() * tau;
                    next.f_cost = next.g_cost + 
                                  params_.lambda_heuristic * 
                                  computeHeuristic(next.pos, next.vel, goal_pos, goal_vel);
                    next.parent_idx = all_states.size() - 1;
                    next.input = u;
                    next.duration = tau;
                    
                    // 状态离散化检查（避免重复访问）
                    int state_key = computeStateKey(next.pos, next.vel);
                    if (closed_set.find(state_key) != closed_set.end()) continue;
                    
                    closed_set[state_key] = next;
                    all_states.push_back(next);
                    open_set.push(next);
                }
            }
        }
        
        return false;  // 搜索失败
    }
    
private:
    // 启发式函数：双积分器最优控制解
    double computeHeuristic(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                            const Eigen::Vector3d& goal_pos, const Eigen::Vector3d& goal_vel) {
        // 简化启发式：距离 + 速度差异
        double dist = (pos - goal_pos).norm();
        double vel_diff = (vel - goal_vel).norm();
        
        // 估计时间下界
        double time_est = dist / params_.max_vel + vel_diff / params_.max_acc;
        
        return time_est;
    }
    
    // 尝试解析扩展（直接连接到目标）
    bool tryAnalyticExpansion(const State& current, 
                               const Eigen::Vector3d& goal_pos,
                               const Eigen::Vector3d& goal_vel,
                               std::vector<Eigen::Vector3d>& path) {
        // 计算最优边值问题解
        // 简化版本：检查直线连接
        if (checkCollisionFree(current.pos, goal_pos)) {
            double dist = (current.pos - goal_pos).norm();
            if (dist < params_.resolution * 5) {
                path.push_back(current.pos);
                path.push_back(goal_pos);
                return true;
            }
        }
        return false;
    }
    
    // 碰撞检查（线段）
    bool checkCollisionFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        int num_checks = static_cast<int>((p1 - p2).norm() / (params_.resolution * 0.5)) + 1;
        
        for (int i = 0; i <= num_checks; i++) {
            double t = static_cast<double>(i) / num_checks;
            Eigen::Vector3d pt = p1 + t * (p2 - p1);
            
            if (dist_func_(pt) < safe_distance_) {
                return false;
            }
        }
        
        return true;
    }
    
    // 状态离散化键值
    int computeStateKey(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) {
        int px = static_cast<int>(pos.x() / params_.resolution);
        int py = static_cast<int>(pos.y() / params_.resolution);
        int pz = static_cast<int>(pos.z() / params_.resolution);
        int vx = static_cast<int>(vel.x() / (params_.max_vel * 0.5));
        int vy = static_cast<int>(vel.y() / (params_.max_vel * 0.5));
        int vz = static_cast<int>(vel.z() / (params_.max_vel * 0.5));
        
        // 简单哈希
        return px + py * 1000 + pz * 1000000 + vx * 10 + vy * 100 + vz * 1000;
    }
};
```

### 5.2 JPS集成（使用jps3d库）

```cpp
// 文件: path_searching/include/jps_wrapper.hpp

#pragma once
#include <jps_planner/jps_planner.h>  // 来自 KumarRobotics/jps3d
#include <Eigen/Dense>
#include <vector>

class JPSWrapper {
private:
    std::shared_ptr<JPS::VoxelMapUtil> map_util_;
    std::unique_ptr<JPS::JPSPlanner3D> planner_;
    
public:
    void setMap(const std::vector<int8_t>& occupancy_data,
                const Eigen::Vector3d& origin,
                const Eigen::Vector3d& dim,
                double resolution) {
        map_util_ = std::make_shared<JPS::VoxelMapUtil>();
        
        // 设置地图参数
        map_util_->setMap(origin, dim, occupancy_data, resolution);
        
        // 初始化规划器
        planner_ = std::make_unique<JPS::JPSPlanner3D>(true);  // true = JPS
        planner_->setMapUtil(map_util_);
    }
    
    bool plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
              std::vector<Eigen::Vector3d>& path) {
        bool success = planner_->plan(start, goal, 1.0);  // 启发式权重
        
        if (success) {
            auto raw_path = planner_->getRawPath();
            path.clear();
            for (const auto& pt : raw_path) {
                path.push_back(Eigen::Vector3d(pt(0), pt(1), pt(2)));
            }
        }
        
        return success;
    }
};
```

---

## 六、kr_mav_control集成

### 6.1 轨迹服务器实现

```cpp
// 文件: traj_server/src/traj_server_node.cpp

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include "bspline_trajectory.hpp"

class TrajectoryServer {
private:
    ros::NodeHandle nh_;
    ros::Publisher pos_cmd_pub_;
    ros::Subscriber odom_sub_;
    ros::Timer cmd_timer_;
    
    BsplineTrajectory trajectory_;
    bool has_trajectory_ = false;
    double start_time_;
    
    // 当前状态
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    
public:
    TrajectoryServer() {
        // 发布位置指令到kr_mav_control
        pos_cmd_pub_ = nh_.advertise<kr_mav_msgs::PositionCommand>(
            "quadrotor/position_cmd", 10);
        
        // 订阅里程计
        odom_sub_ = nh_.subscribe("quadrotor/odom", 10, 
            &TrajectoryServer::odomCallback, this);
        
        // 100Hz指令发布定时器
        cmd_timer_ = nh_.createTimer(ros::Duration(0.01),
            &TrajectoryServer::cmdCallback, this);
    }
    
    void setTrajectory(const BsplineTrajectory& traj) {
        trajectory_ = traj;
        has_trajectory_ = true;
        start_time_ = ros::Time::now().toSec();
        ROS_INFO("Trajectory set, duration: %.2f s", trajectory_.getTotalTime());
    }
    
private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pos_ << msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z;
        current_vel_ << msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z;
    }
    
    void cmdCallback(const ros::TimerEvent& event) {
        if (!has_trajectory_) return;
        
        double t = ros::Time::now().toSec() - start_time_;
        
        if (t > trajectory_.getTotalTime()) {
            // 轨迹结束，保持最终位置
            t = trajectory_.getTotalTime();
            has_trajectory_ = false;
        }
        
        // 采样轨迹
        Eigen::Vector3d pos = trajectory_.getPosition(t);
        Eigen::Vector3d vel = trajectory_.getVelocity(t);
        Eigen::Vector3d acc = trajectory_.getAcceleration(t);
        
        // 发布位置指令
        kr_mav_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        
        cmd.position.x = pos.x();
        cmd.position.y = pos.y();
        cmd.position.z = pos.z();
        
        cmd.velocity.x = vel.x();
        cmd.velocity.y = vel.y();
        cmd.velocity.z = vel.z();
        
        cmd.acceleration.x = acc.x();
        cmd.acceleration.y = acc.y();
        cmd.acceleration.z = acc.z();
        
        // 偏航角保持为0（或从轨迹计算）
        cmd.yaw = 0;
        cmd.yaw_dot = 0;
        
        pos_cmd_pub_.publish(cmd);
    }
};
```

### 6.2 kr_mav_control配置

```yaml
# config/kr_mav_control.yaml

# 位置控制增益（世界坐标系）
gains:
  kp_x: 7.4
  kp_y: 7.4
  kp_z: 10.4
  kd_x: 4.8
  kd_y: 4.8
  kd_z: 6.0

# 姿态控制增益
attitude:
  kR: [1.5, 1.5, 1.0]    # 旋转增益
  kOm: [0.13, 0.13, 0.1]  # 角速度增益

# 物理参数
mass: 0.5  # kg
gravity: 9.81
```

### 6.3 Launch文件配置

```xml
<!-- launch/trajectory_planning.launch -->

<launch>
    <!-- 地图生成器 -->
    <node pkg="map_generator" type="random_forest" name="map_generator" output="screen">
        <param name="map/x_size" value="30.0"/>
        <param name="map/y_size" value="30.0"/>
        <param name="map/z_size" value="3.0"/>
        <param name="map/resolution" value="0.1"/>
        <param name="map/cylinder_ratio" value="0.10"/>
        <param name="map/seed" value="-1"/>  <!-- 随机种子 -->
        <param name="map/inflate_radius" value="0.3"/>
    </node>
    
    <!-- ESDF建图（可选，使用Voxblox） -->
    <node pkg="voxblox_ros" type="esdf_server" name="esdf_server" output="screen">
        <remap from="pointcloud" to="/map_generator/global_cloud"/>
        <param name="tsdf_voxel_size" value="0.10"/>
        <param name="esdf_max_distance_m" value="4.0"/>
        <param name="publish_esdf_map" value="true"/>
    </node>
    
    <!-- 轨迹规划器 -->
    <node pkg="trajectory_planner" type="planner_node" name="planner" output="screen">
        <rosparam file="$(find trajectory_planner)/config/planner.yaml"/>
        <remap from="~odom" to="/quadrotor/odom"/>
        <remap from="~goal" to="/goal"/>
        <remap from="~global_cloud" to="/map_generator/global_cloud"/>
    </node>
    
    <!-- 轨迹服务器 -->
    <node pkg="trajectory_planner" type="traj_server_node" name="traj_server" output="screen">
        <remap from="quadrotor/position_cmd" to="/quadrotor/position_cmd"/>
        <remap from="quadrotor/odom" to="/quadrotor/odom"/>
    </node>
    
    <!-- kr_mav_control -->
    <include file="$(find kr_mav_control)/launch/so3_control.launch">
        <arg name="mav_name" value="quadrotor"/>
        <arg name="mass" value="0.5"/>
    </include>
    
    <!-- RViz可视化 -->
    <node pkg="rviz" type="rviz" name="rviz" 
          args="-d $(find trajectory_planner)/rviz/planner.rviz"/>
</launch>
```

---

## 七、环境建图模块

### 7.1 基于Grid Map的轻量级ESDF（无需Voxblox）

```cpp
// 文件: plan_env/include/grid_map.hpp
// 参考EGO-Planner的轻量级实现

#pragma once
#include <Eigen/Dense>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class GridMap {
public:
    struct Parameters {
        Eigen::Vector3d origin;
        Eigen::Vector3d map_size;
        double resolution = 0.1;
        double inflate_radius = 0.3;
    };
    
private:
    Parameters params_;
    Eigen::Vector3i grid_size_;
    std::vector<int8_t> occupancy_;  // 0: free, 1: occupied
    std::vector<double> distance_;    // 距离场
    
public:
    void initMap(const Parameters& params) {
        params_ = params;
        grid_size_ = ((params_.map_size) / params_.resolution).cast<int>();
        
        int total_size = grid_size_.x() * grid_size_.y() * grid_size_.z();
        occupancy_.resize(total_size, 0);
        distance_.resize(total_size, std::numeric_limits<double>::max());
    }
    
    // 从点云更新地图
    void updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // 清空
        std::fill(occupancy_.begin(), occupancy_.end(), 0);
        std::fill(distance_.begin(), distance_.end(), std::numeric_limits<double>::max());
        
        // 标记占据
        for (const auto& pt : cloud->points) {
            Eigen::Vector3d pos(pt.x, pt.y, pt.z);
            Eigen::Vector3i idx = posToIndex(pos);
            
            if (isInMap(idx)) {
                int linear_idx = indexToLinear(idx);
                occupancy_[linear_idx] = 1;
                distance_[linear_idx] = 0;
            }
        }
        
        // 计算距离场（简化版EDT）
        computeEDT();
    }
    
    // 查询距离
    double getDistance(const Eigen::Vector3d& pos) const {
        Eigen::Vector3i idx = posToIndex(pos);
        if (!isInMap(idx)) return 0;
        
        int linear_idx = indexToLinear(idx);
        return distance_[linear_idx] * params_.resolution;
    }
    
    // 查询距离梯度（数值差分）
    Eigen::Vector3d getGradient(const Eigen::Vector3d& pos) const {
        double eps = params_.resolution;
        Eigen::Vector3d grad;
        
        grad.x() = (getDistance(pos + Eigen::Vector3d(eps, 0, 0)) - 
                    getDistance(pos - Eigen::Vector3d(eps, 0, 0))) / (2 * eps);
        grad.y() = (getDistance(pos + Eigen::Vector3d(0, eps, 0)) - 
                    getDistance(pos - Eigen::Vector3d(0, eps, 0))) / (2 * eps);
        grad.z() = (getDistance(pos + Eigen::Vector3d(0, 0, eps)) - 
                    getDistance(pos - Eigen::Vector3d(0, 0, eps))) / (2 * eps);
        
        return grad.normalized();
    }
    
    // 检查是否碰撞
    bool isOccupied(const Eigen::Vector3d& pos) const {
        return getDistance(pos) < params_.inflate_radius;
    }
    
private:
    Eigen::Vector3i posToIndex(const Eigen::Vector3d& pos) const {
        return ((pos - params_.origin) / params_.resolution).cast<int>();
    }
    
    Eigen::Vector3d indexToPos(const Eigen::Vector3i& idx) const {
        return params_.origin + idx.cast<double>() * params_.resolution;
    }
    
    int indexToLinear(const Eigen::Vector3i& idx) const {
        return idx.x() + idx.y() * grid_size_.x() + 
               idx.z() * grid_size_.x() * grid_size_.y();
    }
    
    bool isInMap(const Eigen::Vector3i& idx) const {
        return idx.x() >= 0 && idx.x() < grid_size_.x() &&
               idx.y() >= 0 && idx.y() < grid_size_.y() &&
               idx.z() >= 0 && idx.z() < grid_size_.z();
    }
    
    // 简化版欧几里得距离变换
    void computeEDT() {
        // 使用简单的BFS近似（生产环境应使用更高效的EDT算法）
        std::queue<Eigen::Vector3i> queue;
        
        // 初始化：障碍物点入队
        for (int x = 0; x < grid_size_.x(); x++) {
            for (int y = 0; y < grid_size_.y(); y++) {
                for (int z = 0; z < grid_size_.z(); z++) {
                    Eigen::Vector3i idx(x, y, z);
                    int linear_idx = indexToLinear(idx);
                    if (occupancy_[linear_idx] == 1) {
                        queue.push(idx);
                    }
                }
            }
        }
        
        // BFS传播
        std::vector<Eigen::Vector3i> neighbors = {
            {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
        };
        
        while (!queue.empty()) {
            Eigen::Vector3i current = queue.front();
            queue.pop();
            
            int current_linear = indexToLinear(current);
            double current_dist = distance_[current_linear];
            
            for (const auto& d : neighbors) {
                Eigen::Vector3i neighbor = current + d;
                
                if (!isInMap(neighbor)) continue;
                
                int neighbor_linear = indexToLinear(neighbor);
                double new_dist = current_dist + 1;
                
                if (new_dist < distance_[neighbor_linear]) {
                    distance_[neighbor_linear] = new_dist;
                    queue.push(neighbor);
                }
            }
        }
    }
};
```

### 7.2 地图接口封装

```cpp
// 文件: plan_env/include/map_interface.hpp

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "grid_map.hpp"

class MapInterface {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    
    GridMap grid_map_;
    bool map_ready_ = false;
    
public:
    MapInterface() {
        // 初始化地图参数
        GridMap::Parameters params;
        params.origin = Eigen::Vector3d(-15, -15, 0);
        params.map_size = Eigen::Vector3d(30, 30, 3);
        params.resolution = 0.1;
        params.inflate_radius = 0.3;
        
        grid_map_.initMap(params);
        
        // 订阅点云
        cloud_sub_ = nh_.subscribe("/map_generator/global_cloud", 1,
            &MapInterface::cloudCallback, this);
    }
    
    bool isMapReady() const { return map_ready_; }
    
    double getDistance(const Eigen::Vector3d& pos) const {
        return grid_map_.getDistance(pos);
    }
    
    Eigen::Vector3d getGradient(const Eigen::Vector3d& pos) const {
        return grid_map_.getGradient(pos);
    }
    
    bool isOccupied(const Eigen::Vector3d& pos) const {
        return grid_map_.isOccupied(pos);
    }
    
private:
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        grid_map_.updateMap(cloud);
        map_ready_ = true;
        
        ROS_INFO_ONCE("Map received and processed");
    }
};
```

---

## 八、状态机与主规划节点

### 8.1 规划管理器实现

```cpp
// 文件: plan_manage/src/planner_manager.cpp

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include "map_interface.hpp"
#include "kinodynamic_astar.hpp"
#include "bspline_optimizer.hpp"
#include "bspline_trajectory.hpp"

enum class PlannerState {
    IDLE,
    WAIT_GOAL,
    PLANNING,
    EXECUTING,
    REPLANNING
};

class PlannerManager {
private:
    ros::NodeHandle nh_;
    
    // 订阅/发布
    ros::Subscriber odom_sub_, goal_sub_;
    ros::Publisher traj_pub_, vis_pub_;
    ros::Timer planning_timer_, safety_timer_;
    
    // 模块
    std::shared_ptr<MapInterface> map_;
    std::shared_ptr<KinodynamicAstar> path_finder_;
    std::shared_ptr<BsplineOptimizer> optimizer_;
    
    // 状态
    PlannerState state_ = PlannerState::IDLE;
    Eigen::Vector3d current_pos_, current_vel_;
    Eigen::Vector3d goal_pos_;
    bool has_goal_ = false;
    
    // 当前轨迹
    BsplineTrajectory current_traj_;
    double traj_start_time_;
    
public:
    PlannerManager() {
        // 初始化模块
        map_ = std::make_shared<MapInterface>();
        path_finder_ = std::make_shared<KinodynamicAstar>();
        optimizer_ = std::make_shared<BsplineOptimizer>();
        
        // 设置距离函数
        path_finder_->setDistanceFunction(
            [this](const Eigen::Vector3d& p) { return map_->getDistance(p); });
        path_finder_->setSafeDistance(0.5);
        
        // 订阅
        odom_sub_ = nh_.subscribe("odom", 10, &PlannerManager::odomCallback, this);
        goal_sub_ = nh_.subscribe("goal", 10, &PlannerManager::goalCallback, this);
        
        // 发布
        vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization", 10);
        
        // 定时器
        planning_timer_ = nh_.createTimer(ros::Duration(0.1),  // 10Hz
            &PlannerManager::planningCallback, this);
        safety_timer_ = nh_.createTimer(ros::Duration(0.02),   // 50Hz
            &PlannerManager::safetyCallback, this);
        
        state_ = PlannerState::WAIT_GOAL;
        ROS_INFO("Planner initialized, waiting for goal...");
    }
    
private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pos_ << msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z;
        current_vel_ << msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z;
    }
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_pos_ << msg->pose.position.x,
                     msg->pose.position.y,
                     msg->pose.position.z;
        has_goal_ = true;
        state_ = PlannerState::PLANNING;
        ROS_INFO("New goal received: [%.2f, %.2f, %.2f]", 
                 goal_pos_.x(), goal_pos_.y(), goal_pos_.z());
    }
    
    void planningCallback(const ros::TimerEvent& event) {
        if (state_ != PlannerState::PLANNING && state_ != PlannerState::REPLANNING) {
            return;
        }
        
        if (!map_->isMapReady()) {
            ROS_WARN_THROTTLE(1.0, "Waiting for map...");
            return;
        }
        
        ros::Time t_start = ros::Time::now();
        
        // Step 1: 前端路径搜索
        std::vector<Eigen::Vector3d> init_path;
        bool search_success = path_finder_->search(
            current_pos_, current_vel_, goal_pos_, Eigen::Vector3d::Zero(), init_path);
        
        if (!search_success) {
            ROS_WARN("Path search failed!");
            state_ = PlannerState::WAIT_GOAL;
            return;
        }
        
        ROS_INFO("Path search succeeded, %zu waypoints", init_path.size());
        
        // Step 2: 初始化B样条轨迹
        BsplineTrajectory traj;
        double path_length = 0;
        for (size_t i = 1; i < init_path.size(); i++) {
            path_length += (init_path[i] - init_path[i-1]).norm();
        }
        double init_time = path_length / 2.0 * 1.5;  // 保守时间分配
        traj.initFromPath(init_path, init_time);
        
        // Step 3: B样条优化
        optimizer_->setTrajectory(&traj);
        optimizer_->setDistanceFunction(
            [this](const Eigen::Vector3d& p) { return map_->getDistance(p); });
        optimizer_->setGradientFunction(
            [this](const Eigen::Vector3d& p) { return map_->getGradient(p); });
        optimizer_->setGuidePath(init_path);
        
        BsplineOptimizer::Parameters opt_params;
        opt_params.lambda_smooth = 1.0;
        opt_params.lambda_collision = 10.0;
        opt_params.lambda_feasibility = 1.0;
        opt_params.max_vel = 3.0;
        opt_params.max_acc = 6.0;
        optimizer_->setParameters(opt_params);
        
        bool opt_success = optimizer_->optimize();
        
        if (!opt_success) {
            ROS_WARN("Trajectory optimization failed!");
            state_ = PlannerState::WAIT_GOAL;
            return;
        }
        
        double planning_time = (ros::Time::now() - t_start).toSec() * 1000;
        ROS_INFO("Planning succeeded in %.2f ms", planning_time);
        
        // 更新当前轨迹
        current_traj_ = traj;
        traj_start_time_ = ros::Time::now().toSec();
        
        // 发布可视化
        publishVisualization();
        
        state_ = PlannerState::EXECUTING;
    }
    
    void safetyCallback(const ros::TimerEvent& event) {
        if (state_ != PlannerState::EXECUTING) return;
        
        // 检查当前轨迹是否安全
        double t = ros::Time::now().toSec() - traj_start_time_;
        
        // 检查未来一段时间的轨迹
        for (double dt = 0; dt < 2.0; dt += 0.1) {
            if (t + dt > current_traj_.getTotalTime()) break;
            
            Eigen::Vector3d pos = current_traj_.getPosition(t + dt);
            if (map_->isOccupied(pos)) {
                ROS_WARN("Collision detected on trajectory, replanning...");
                state_ = PlannerState::REPLANNING;
                return;
            }
        }
        
        // 检查跟踪误差
        Eigen::Vector3d traj_pos = current_traj_.getPosition(t);
        if ((current_pos_ - traj_pos).norm() > 0.5) {
            ROS_WARN("Large tracking error, replanning...");
            state_ = PlannerState::REPLANNING;
            return;
        }
        
        // 检查是否到达目标
        if (t >= current_traj_.getTotalTime()) {
            if ((current_pos_ - goal_pos_).norm() < 0.3) {
                ROS_INFO("Goal reached!");
                state_ = PlannerState::WAIT_GOAL;
                has_goal_ = false;
            }
        }
    }
    
    void publishVisualization() {
        visualization_msgs::MarkerArray markers;
        
        // 轨迹可视化
        visualization_msgs::Marker traj_marker;
        traj_marker.header.frame_id = "world";
        traj_marker.header.stamp = ros::Time::now();
        traj_marker.ns = "trajectory";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::Marker::ADD;
        traj_marker.scale.x = 0.05;
        traj_marker.color.r = 0.0;
        traj_marker.color.g = 1.0;
        traj_marker.color.b = 0.0;
        traj_marker.color.a = 1.0;
        
        for (double t = 0; t < current_traj_.getTotalTime(); t += 0.05) {
            Eigen::Vector3d pos = current_traj_.getPosition(t);
            geometry_msgs::Point pt;
            pt.x = pos.x();
            pt.y = pos.y();
            pt.z = pos.z();
            traj_marker.points.push_back(pt);
        }
        
        markers.markers.push_back(traj_marker);
        
        // 控制点可视化
        visualization_msgs::Marker ctrl_marker;
        ctrl_marker.header = traj_marker.header;
        ctrl_marker.ns = "control_points";
        ctrl_marker.id = 1;
        ctrl_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        ctrl_marker.action = visualization_msgs::Marker::ADD;
        ctrl_marker.scale.x = 0.1;
        ctrl_marker.scale.y = 0.1;
        ctrl_marker.scale.z = 0.1;
        ctrl_marker.color.r = 1.0;
        ctrl_marker.color.g = 0.0;
        ctrl_marker.color.b = 0.0;
        ctrl_marker.color.a = 1.0;
        
        for (const auto& cp : current_traj_.getControlPoints()) {
            geometry_msgs::Point pt;
            pt.x = cp.x();
            pt.y = cp.y();
            pt.z = cp.z();
            ctrl_marker.points.push_back(pt);
        }
        
        markers.markers.push_back(ctrl_marker);
        
        vis_pub_.publish(markers);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_manager");
    
    PlannerManager planner;
    
    ros::spin();
    return 0;
}
```

---

## 九、项目文件结构

```
trajectory_planner/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── planner.yaml
│   ├── bspline_optimizer.yaml
│   └── kr_mav_control.yaml
├── launch/
│   ├── trajectory_planning.launch
│   └── simulation.launch
├── rviz/
│   └── planner.rviz
├── include/
│   └── trajectory_planner/
│       ├── bspline_base.hpp
│       ├── bspline_trajectory.hpp
│       ├── bspline_optimizer.hpp
│       ├── kinodynamic_astar.hpp
│       ├── jps_wrapper.hpp
│       ├── grid_map.hpp
│       ├── map_interface.hpp
│       └── corridor_gen.hpp
├── src/
│   ├── planner_manager.cpp
│   └── traj_server_node.cpp
└── third_party/
    └── lbfgs_lite/
        └── lbfgs.hpp
```

---

## 十、依赖安装与编译

### 10.1 系统依赖

```bash
# 基础依赖
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install ros-noetic-pcl-ros ros-noetic-tf2-ros
sudo apt install libeigen3-dev libnlopt-dev

# 可选：Voxblox（如果使用）
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/voxblox.git
git clone https://github.com/ethz-asl/voxblox_ros.git

# JPS3D
git clone https://github.com/KumarRobotics/jps3d.git

# 指定仓库
git clone https://github.com/yuwei-wu/map_generator.git
git clone https://github.com/KumarRobotics/kr_mav_control.git
```

### 10.2 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  kr_mav_msgs
  pcl_ros
  tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs kr_mav_msgs
)

include_directories(
  include
  third_party
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 主规划节点
add_executable(planner_node src/planner_manager.cpp)
target_link_libraries(planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# 轨迹服务器
add_executable(traj_server_node src/traj_server_node.cpp)
target_link_libraries(traj_server_node ${catkin_LIBRARIES})
```

---

## 十一、开发检查清单

### 第一阶段 - 环境配置（1周）
- [ ] 安装ROS Noetic及catkin工作空间
- [ ] 克隆map_generator、kr_mav_control
- [ ] 验证map_generator发布点云
- [ ] 验证kr_mav_control仿真环境

### 第二阶段 - 地图模块（1-2周）
- [ ] 实现GridMap类
- [ ] 实现距离查询和梯度查询
- [ ] 订阅点云并更新地图
- [ ] 测试碰撞检测

### 第三阶段 - 前端搜索（1-2周）
- [ ] 实现运动学A*
- [ ] 添加运动基元生成
- [ ] 实现启发式函数
- [ ] 测试路径搜索质量

### 第四阶段 - 轨迹优化（2周）
- [ ] 实现B样条类
- [ ] 实现平滑代价和梯度
- [ ] 实现碰撞代价（EGO方法）
- [ ] 实现可行性代价
- [ ] 集成LBFGS-Lite
- [ ] 验证优化结果

### 第五阶段 - 控制集成（1周）
- [ ] 实现轨迹服务器
- [ ] 配置kr_mav_control参数
- [ ] 测试闭环跟踪

### 第六阶段 - 系统集成（1-2周）
- [ ] 实现状态机
- [ ] 添加重规划逻辑
- [ ] 添加RViz可视化
- [ ] 完整系统测试
- [ ] 参数调优

---

## 十二、参考资源

### 核心论文
1. Mellinger & Kumar (2011) - Minimum Snap Trajectory Generation
2. Liu et al. (2017) - Planning Dynamically Feasible Trajectories (Safe Flight Corridors)
3. Zhou et al. (2019) - Fast-Planner
4. Zhou et al. (2021) - EGO-Planner
5. Wang et al. (2022) - GCOPTER / MINCO

### 开源仓库
- EGO-Planner: https://github.com/ZJU-FAST-Lab/ego-planner
- Fast-Planner: https://github.com/HKUST-Aerial-Robotics/Fast-Planner
- GCOPTER: https://github.com/ZJU-FAST-Lab/GCOPTER
- LBFGS-Lite: https://github.com/ZJU-FAST-Lab/LBFGS-Lite
- jps3d: https://github.com/KumarRobotics/jps3d
- kr_mav_control: https://github.com/KumarRobotics/kr_mav_control
- map_generator: https://github.com/yuwei-wu/map_generator

---

*本技术路线提供了构建完整无人机轨迹规划系统所需的架构基础和实现细节。模块化设计允许增量开发和测试，而指定的接口确保与kr_mav_control栈和map_generator地图生成器的兼容性。*
