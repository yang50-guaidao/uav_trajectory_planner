# Technical roadmap for UAV trajectory optimization in cluttered environments

Building a real-time quadrotor trajectory planner that navigates cluttered spaces requires integrating five core systems: environment mapping, path search, trajectory optimization, safe corridor generation, and low-level control. This roadmap provides implementation-ready specifications for a ROS1 Noetic system using B-spline trajectories, the specified kr_mav_control stack, and kr_param_map generator.

The architecture follows the proven Fast-Planner/EGO-Planner paradigm: a front-end kinodynamic search produces an initial path, which the back-end B-spline optimizer refines for smoothness, collision avoidance, and dynamic feasibility. Planning times under **5ms** are achievable using ESDF-free gradient computation from EGO-Planner, while MINCO trajectory representation from GCOPTER offers superior optimality for less time-critical applications.

## System architecture spans five interconnected modules

The planning pipeline processes sensor data through a modular stack where each component has clearly defined interfaces. The kr_param_map generator publishes obstacle point clouds on `/structure_map/global_cloud` as `sensor_msgs/PointCloud2`, which feeds directly into the ESDF computation layer. The planner's output connects to kr_mav_control through position commands that the SO3 controller converts to attitude and thrust.

```
┌─────────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  kr_param_map   │───►│  plan_env   │───►│path_search  │───►│ bspline_opt │
│  (PointCloud2)  │    │  (ESDF)     │    │(Kinodyn A*) │    │ (L-BFGS)    │
└─────────────────┘    └─────────────┘    └─────────────┘    └──────┬──────┘
                                                                    │
┌─────────────────┐    ┌─────────────┐    ┌─────────────┐          │
│ kr_mav_control  │◄───│ SO3Control  │◄───│ traj_server │◄─────────┘
│ (motor cmds)    │    │(PositionCmd)│    │(100Hz sample)│
└─────────────────┘    └─────────────┘    └─────────────┘
```

**Module decomposition** mirrors Fast-Planner's proven structure: `plan_env` handles ESDF mapping and distance queries, `path_searching` implements kinodynamic A* and JPS, `bspline_opt` runs gradient-based trajectory optimization, `plan_manage` orchestrates the pipeline with state machine logic, and `traj_server` samples trajectories at **100Hz** for the controller.

## B-spline trajectory optimization uses gradient descent on control points

Uniform cubic B-splines represent trajectories through control points **Q** = {Q₀, Q₁, ..., Qₙ} with the **convex hull property** guaranteeing that if all control points lie within a convex safe region, the entire trajectory segment remains collision-free. This property enables converting obstacle avoidance into linear constraints on control points.

**The optimization minimizes a composite cost function:**

```
J_total = λ_s·J_smooth + λ_c·J_collision + λ_d·J_feasibility
```

The **smoothness cost** penalizes jerk through second differences of control points:
```cpp
J_smooth = Σᵢ ||Qᵢ₊₁ - 2Qᵢ + Qᵢ₋₁||² / Δt⁴

∂J_smooth/∂Qᵢ = 2·(6Qᵢ - 4Qᵢ₊₁ - 4Qᵢ₋₁ + Qᵢ₊₂ + Qᵢ₋₂) / Δt⁴
```

The **collision cost** traditionally uses ESDF gradients, but EGO-Planner's breakthrough eliminates this expensive computation. Instead, when a trajectory segment collides, the optimizer finds the closest point on a collision-free guiding path and computes:
```cpp
∂J_collision/∂Qᵢ = 2·w_c·(Qᵢ - p_guide)  // No ESDF needed
```

The **feasibility cost** enforces velocity and acceleration limits derived from B-spline derivatives:
```cpp
vᵢ = (Qᵢ₊₁ - Qᵢ) / Δt        // Velocity from control point differences
aᵢ = (Qᵢ₊₁ - 2Qᵢ + Qᵢ₋₁) / Δt²  // Acceleration

J_feasibility = Σᵢ max(0, ||vᵢ|| - v_max)² + max(0, ||aᵢ|| - a_max)²
```

**Typical optimization parameters** (from EGO-Planner):
| Parameter | Value | Description |
|-----------|-------|-------------|
| `lambda_smooth` | 1.0 | Smoothness weight |
| `lambda_collision` | 10.0 | Collision avoidance weight |
| `lambda_feasibility` | 1.0 | Dynamic constraint weight |
| `max_vel` | 3.0 m/s | Maximum velocity |
| `max_acc` | 6.0 m/s² | Maximum acceleration |
| `ctrl_pt_dist` | 0.4-0.5 m | Control point spacing |
| `max_iteration` | 100 | L-BFGS iterations |

## MINCO representation offers superior optimality over classic B-splines

MINCO (Minimum Control) from ZJU-FAST-Lab's GCOPTER represents trajectories through **sparse parameters**: intermediate waypoints **q** = {q₁, ..., qₘ} and segment durations **T** = {T₁, ..., Tₘ}. Polynomial coefficients are analytically determined via a linear mapping C = M(q, T), enabling **joint spatial-temporal optimization** with O(M) gradient complexity.

For snap minimization (s=4), each segment uses a 7th-order polynomial with coefficients computed from boundary conditions. The key advantage over B-splines is **diffeomorphic constraint elimination** - constraints like corridor containment are eliminated through smooth variable transformations rather than penalty terms, avoiding local minima.

**MINCO implementation class structure** (from GCOPTER):
```cpp
template <int D>  // D = dimension (3 for position)
class MINCO_S4 {
    Eigen::MatrixXd innerPoints;  // m waypoints × D dimensions
    Eigen::VectorXd durations;     // m segment times
    
    void setConditions(start_pos, start_vel, end_pos, end_vel, num_pieces);
    void generateTraj();  // Compute polynomial coefficients
    void getGrad(grad_inner_pts, grad_durations);  // Analytical gradients
};
```

**When to choose each representation:**
- **B-spline**: Real-time replanning (<5ms), simpler implementation, local modifications
- **MINCO**: Higher trajectory quality, time-optimal paths, corridor-constrained problems

## Safe flight corridor generation converts obstacle avoidance to linear constraints

The **IRIS algorithm** (Iterative Regional Inflation by Semidefinite Programming) generates maximal convex obstacle-free regions through alternating optimization: first compute separating hyperplanes from an inflating ellipsoid to each obstacle, then solve an SDP to find the maximum-volume inscribed ellipsoid. Convergence typically occurs in **3-10 iterations**.

For real-time applications, **FIRI** (Fast Iterative Regional Inflation) replaces the SDP with Second-Order Cone Programming, achieving **~10x speedup** with similar volume quality.

**Corridor constraint formulation:**
Each polytope corridor P is defined by half-planes: A·x ≤ b. The B-spline convex hull property ensures collision-free trajectories by constraining control points:
```
A·Qᵢ ≤ b - ε,  for all control points in segment
```
where ε provides a safety margin. This converts the non-convex obstacle avoidance problem into **linear inequality constraints** that QP solvers like OSQP handle efficiently.

**ESDF computation through Voxblox** provides distance and gradient queries essential for gradient-based optimization. Key configuration:
```yaml
tsdf_voxel_size: 0.10      # Resolution (0.05-0.20m typical)
esdf_max_distance: 4.0     # Maximum distance computed
truncation_distance: 0.4   # TSDF truncation (2-4× voxel_size)
```

**Voxblox C++ query API:**
```cpp
voxblox::EsdfServer esdf_server_;

double getDistance(const Eigen::Vector3d& pos) {
    double distance;
    esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(pos, &distance);
    return distance;
}

bool getDistanceAndGradient(const Eigen::Vector3d& pos,
                            double* dist, Eigen::Vector3d* grad) {
    return esdf_server_.getEsdfMapPtr()->
        getDistanceAndGradientAtPosition(pos, dist, grad);
}
```

## Kinodynamic A* provides dynamically feasible initial paths

The front-end search operates in 6D state space **[x, y, z, vx, vy, vz]** using motion primitives generated by discretizing acceleration inputs. The heuristic uses the optimal-control solution for a double integrator, ensuring admissibility.

**Motion primitive generation:**
```cpp
// Discretize control space
controls = {-a_max, 0, a_max}  // for each axis
time_durations = {0.05, 0.1, 0.2}  // seconds

// Forward propagation (double integrator)
x_new = x + v*τ + 0.5*u*τ²
v_new = v + u*τ
```

**Critical optimization: Analytic expansion** - when approaching the goal, the planner attempts to connect directly using an optimal boundary value problem solution. If the resulting trajectory is collision-free, it shortcuts the remaining search, dramatically reducing computation.

**JPS (Jump Point Search)** accelerates geometric path finding by **10-100x** over A* through intelligent pruning. The jps3d library from KumarRobotics provides ready-to-use 3D implementation:
```cpp
JPSPlanner3D planner(true);  // true = JPS, false = A*
planner.setMapUtil(map_util);
planner.plan(start, goal, 1.0);  // heuristic weight
auto path = planner.getRawPath();
```

## kr_mav_control integration requires SO3Command message publishing

The kr_mav_control stack from Kumar Robotics uses an SO3-based controller that accepts `kr_mav_msgs/SO3Command` messages. The trajectory planner publishes position commands that the SO3 controller converts to attitude and thrust.

**SO3Command message definition:**
```msg
Header header
geometry_msgs/Quaternion orientation  # Desired attitude
geometry_msgs/Vector3 force           # Desired force [0, 0, thrust]
float64[3] kR                         # Rotation gains
float64[3] kOm                        # Angular velocity gains
kr_mav_msgs/Aux aux                   # Auxiliary data
```

**Controller configuration parameters:**
```yaml
# Position gains (world frame)
kp_x: 7.4    kp_y: 7.4    kp_z: 10.4
# Velocity/derivative gains
kd_x: 4.8    kd_y: 4.8    kd_z: 6.0
# Mass (critical for thrust computation)
mass: 0.5   # kg
```

**Integration pattern:** The trajectory server samples the B-spline at 100Hz, publishing `geometry_msgs/PositionCommand` or directly computing SO3Command. The MAV Manager provides services for takeoff, landing, and waypoint navigation.

**ROS topic structure (namespaced by robot):**
| Topic | Type | Purpose |
|-------|------|---------|
| `quadrotor/position_cmd` | kr_mav_msgs/PositionCommand | Position setpoint |
| `quadrotor/so3_cmd` | kr_mav_msgs/SO3Command | Direct attitude command |
| `quadrotor/odom` | nav_msgs/Odometry | State feedback |

## Map generator provides configurable obstacle environments

The kr_param_map (evolved from yuwei-wu/map_generator) publishes point cloud maps suitable for planning evaluation. Configuration supports random generation, image-based maps, and rosbag replay.

**Structure map parameters:**
```xml
<param name="map/x_size" value="30.0"/>
<param name="map/y_size" value="30.0"/>
<param name="map/z_size" value="3.0"/>
<param name="map/resolution" value="0.1"/>
<param name="map/cylinder_ratio" value="0.10"/>  <!-- Obstacle density -->
<param name="map/inflate_radius" value="0.3"/>   <!-- Robot radius -->
```

**Published topics:**
- `/structure_map/global_cloud` (sensor_msgs/PointCloud2) - Main obstacle point cloud
- `/structure_map/change_map` (std_msgs/Bool) - Trigger regeneration

The point cloud feeds directly into Voxblox for TSDF/ESDF generation, or can be processed by EGO-Planner's lightweight grid map that bypasses full ESDF computation.

## Mathematical libraries enable efficient optimization

**L-BFGS** is the optimizer of choice for trajectory planning due to its O(mn) memory efficiency and superlinear convergence. ZJU-FAST-Lab's **LBFGS-Lite** provides a header-only C++ implementation with Lewis-Overton line search for robustness on non-smooth functions.

```cpp
#include "lbfgs.hpp"

double costFunction(void* instance, const double* x, double* grad, int n) {
    double cost = computeCost(x);
    computeGradient(x, grad);
    return cost;
}

lbfgs_parameter_t param;
lbfgs_parameter_init(&param);
param.epsilon = 1e-6;
param.max_iterations = 100;
int ret = lbfgs(n, x, &fx, costFunction, nullptr, nullptr, &param);
```

**OSQP** handles QP subproblems for corridor-constrained optimization:
```python
# min (1/2)x'Px + q'x  s.t. l <= Ax <= u
prob = osqp.OSQP()
prob.setup(P, q, A, l, u, eps_abs=1e-3)
res = prob.solve()
```

**Eigen** provides the matrix operations for B-spline evaluation:
```cpp
// Cubic B-spline basis matrix
Eigen::Matrix4d M_cubic;
M_cubic << -1,  3, -3,  1,
            3, -6,  3,  0,
           -3,  0,  3,  0,
            1,  4,  1,  0;
M_cubic /= 6.0;

// Evaluate at parameter t
Eigen::Vector4d T_vec;
T_vec << t*t*t, t*t, t, 1;
Eigen::Vector3d point = T_vec.transpose() * M_cubic * control_points;
```

## State machine orchestrates the planning pipeline

The planner operates through five states with well-defined transitions:

```
IDLE ──[goal_recv]──► PLANNING ──[plan_success]──► OPTIMIZING
  ▲                                                    │
  │                                               [opt_done]
  │                                                    ▼
  └───[reach_goal]─── EXECUTING ◄──────────────────────┘
                          │
                   [collision_detected]
                          ▼
                     REPLANNING
```

**Replanning triggers:**
- Collision detected on current trajectory
- Tracking error exceeds threshold (typically 0.5m)
- New obstacle appears in planned path
- Goal position changes

**Callback organization:**
```cpp
class PlannerNode {
    ros::Subscriber odom_sub_, goal_sub_, cloud_sub_;
    ros::Publisher traj_pub_, vis_pub_;
    ros::Timer planning_timer_, safety_timer_;
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void planningCallback(const ros::TimerEvent&);  // 10Hz replanning
    void safetyCallback(const ros::TimerEvent&);     // 50Hz collision check
};
```

## Open-source implementations provide battle-tested starting points

| Repository | Best For | Planning Time | License |
|------------|----------|---------------|---------|
| **EGO-Planner** | Real-time replanning | ~1ms | GPLv3 |
| **Fast-Planner** | Full-featured baseline | ~10ms | GPLv3 |
| **GCOPTER** | Optimal trajectories | ~1ms | MIT |
| **mav_trajectory_generation** | Polynomial trajectories | ~10ms | Apache 2.0 |

**Recommended approach:** Start with EGO-Planner's architecture for rapid development, replace the optimization backend with MINCO from GCOPTER for higher trajectory quality, and use mav_trajectory_generation's feasibility checking utilities.

**Reusable components with permissive licenses:**
- LBFGS-Lite (MIT) - Header-only optimizer
- MINCO trajectory class (MIT) - Optimal trajectory representation
- FIRI corridor generation (MIT) - Fast convex decomposition
- FeasibilityAnalytic from ETH (Apache 2.0) - Constraint checking

## Common pitfalls have proven solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| Search timeout in cluttered spaces | Resolution too fine | Use JPS for initial path, reduce kinodynamic resolution |
| Trajectory oscillates near obstacles | Collision weight too low | Increase `lambda_collision` to 10-20 |
| Optimizer produces infeasible trajectory | Time allocation too aggressive | Enable iterative time adjustment |
| ESDF gradient discontinuities | Trilinear interpolation artifacts | Use smoothed gradient with larger threshold (0.6m) |
| Replanning instability | Too frequent triggering | Add hysteresis, increase replan threshold |
| Memory overflow | Large map + fine resolution | Use spatial hashing, limit ESDF range to 4m |

## Implementation checklist summarizes the development path

**Phase 1 - Environment Setup:**
- [ ] Install ROS Noetic with catkin workspace
- [ ] Clone kr_param_map, kr_mav_control, Voxblox
- [ ] Verify map generation → point cloud → ESDF pipeline

**Phase 2 - Front-end Search:**
- [ ] Implement kinodynamic A* with configurable primitives
- [ ] Add JPS for geometric initialization
- [ ] Test path quality in generated environments

**Phase 3 - Trajectory Optimization:**
- [ ] Implement uniform B-spline class with Eigen
- [ ] Add smoothness, collision, feasibility costs
- [ ] Integrate LBFGS-Lite optimizer
- [ ] Validate against max velocity/acceleration constraints

**Phase 4 - Control Integration:**
- [ ] Configure kr_mav_control gains for your platform
- [ ] Implement trajectory server with 100Hz sampling
- [ ] Test closed-loop tracking in simulation

**Phase 5 - System Integration:**
- [ ] Implement state machine with replanning logic
- [ ] Add visualization (RViz markers for trajectory, corridors)
- [ ] Tune parameters for target environment density

**Dependencies to install:**
```bash
sudo apt install ros-noetic-mavros ros-noetic-pcl-ros libeigen3-dev libnlopt-dev
# Build Voxblox from source for latest ESDF features
cd ~/catkin_ws/src && git clone https://github.com/ethz-asl/voxblox.git
catkin build
```

This roadmap provides the architectural foundation and implementation details needed to build a complete UAV trajectory planning system. The modular design allows incremental development and testing, while the specified interfaces ensure compatibility with the kr_mav_control stack and kr_param_map generator.