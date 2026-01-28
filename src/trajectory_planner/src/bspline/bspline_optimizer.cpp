#include "trajectory_planner/bspline/bspline_optimizer.hpp"
#include <algorithm>

namespace trajectory_planner {

void BsplineOptimizer::init(ros::NodeHandle& nh) {
    nh.param("bspline_optimizer/lambda_smooth", config_.lambda_smooth, 10.0);
    nh.param("bspline_optimizer/max_vel", config_.max_vel, 3.0);
    nh.param("bspline_optimizer/max_acc", config_.max_acc, 6.0);
    nh.param("bspline_optimizer/control_point_dist", config_.control_point_dist, 0.4);
    nh.param("bspline_optimizer/max_iterations", config_.max_iterations, 1000);
    nh.param("bspline_optimizer/eps_abs", config_.eps_abs, 1e-4);
    
    ROS_INFO("[BsplineOptimizer] Initialized with lambda_smooth=%.1f, max_vel=%.1f, max_acc=%.1f",
             config_.lambda_smooth, config_.max_vel, config_.max_acc);
}

void BsplineOptimizer::setInitialPath(const std::vector<Eigen::Vector3d>& path, double total_time) {
    if (path.size() < 2) {
        ROS_WARN("[BsplineOptimizer] Path too short");
        return;
    }
    
    // Resample path to have uniform control point spacing
    double path_length = 0;
    for (size_t i = 1; i < path.size(); i++) {
        path_length += (path[i] - path[i-1]).norm();
    }
    
    int num_ctrl_pts = std::max(6, static_cast<int>(path_length / config_.control_point_dist) + 1);
    double segment_length = path_length / (num_ctrl_pts - 1);
    
    // Resample path
    std::vector<Eigen::Vector3d> ctrl_pts;
    ctrl_pts.push_back(path[0]);
    
    double accumulated_dist = 0;
    double target_dist = segment_length;
    
    for (size_t i = 1; i < path.size(); i++) {
        double seg_dist = (path[i] - path[i-1]).norm();
        while (accumulated_dist + seg_dist >= target_dist && ctrl_pts.size() < static_cast<size_t>(num_ctrl_pts - 1)) {
            double ratio = (target_dist - accumulated_dist) / seg_dist;
            Eigen::Vector3d pt = path[i-1] + ratio * (path[i] - path[i-1]);
            ctrl_pts.push_back(pt);
            target_dist += segment_length;
        }
        accumulated_dist += seg_dist;
    }
    ctrl_pts.push_back(path.back());
    
    // For cubic B-spline, add 2 extra control points at start and end
    // to satisfy boundary conditions (zero velocity at endpoints)
    ctrl_pts.insert(ctrl_pts.begin(), ctrl_pts[0]);
    ctrl_pts.insert(ctrl_pts.begin(), ctrl_pts[0]);
    ctrl_pts.push_back(ctrl_pts.back());
    ctrl_pts.push_back(ctrl_pts.back());
    
    double dt = total_time / (ctrl_pts.size() - 3);  // For cubic B-spline
    trajectory_.setControlPoints(ctrl_pts, dt);
    
    ROS_INFO("[BsplineOptimizer] Initialized with %zu control points, dt=%.3f", ctrl_pts.size(), dt);
}

void BsplineOptimizer::setCorridors(const std::vector<Polyhedron>& corridors) {
    corridors_ = corridors;
}

bool BsplineOptimizer::optimize() {
    auto& ctrl_pts = trajectory_.getControlPoints();
    int n = ctrl_pts.size();
    
    if (n < 6) {
        ROS_WARN("[BsplineOptimizer] Not enough control points for optimization");
        return false;
    }
    
    // Variables: 3n (x, y, z for each control point)
    // We only optimize the inner control points (excluding boundary constraints)
    int n_vars = 3 * n;
    
    // Build cost matrices
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);
    buildSmoothnessCost(P, q);
    
    // Build constraint matrices
    Eigen::SparseMatrix<double> A;
    Eigen::VectorXd l, u;
    buildConstraints(A, l, u);
    
    if (A.rows() == 0) {
        ROS_WARN("[BsplineOptimizer] No constraints built, using initial trajectory");
        return true;
    }
    
    // Setup OSQP solver
    solver_.clearSolver();
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();
    
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setMaxIteration(config_.max_iterations);
    solver_.settings()->setAbsoluteTolerance(config_.eps_abs);
    solver_.settings()->setRelativeTolerance(config_.eps_abs);
    
    solver_.data()->setNumberOfVariables(n_vars);
    solver_.data()->setNumberOfConstraints(A.rows());
    
    if (!solver_.data()->setHessianMatrix(P)) {
        ROS_ERROR("[BsplineOptimizer] Failed to set Hessian matrix");
        return false;
    }
    
    if (!solver_.data()->setGradient(q)) {
        ROS_ERROR("[BsplineOptimizer] Failed to set gradient");
        return false;
    }
    
    if (!solver_.data()->setLinearConstraintsMatrix(A)) {
        ROS_ERROR("[BsplineOptimizer] Failed to set constraint matrix");
        return false;
    }
    
    if (!solver_.data()->setLowerBound(l)) {
        ROS_ERROR("[BsplineOptimizer] Failed to set lower bounds");
        return false;
    }
    
    if (!solver_.data()->setUpperBound(u)) {
        ROS_ERROR("[BsplineOptimizer] Failed to set upper bounds");
        return false;
    }
    
    if (!solver_.initSolver()) {
        ROS_ERROR("[BsplineOptimizer] Failed to initialize OSQP solver");
        return false;
    }
    
    // Set initial guess (warm start)
    Eigen::Matrix<c_float, Eigen::Dynamic, 1> x0(n_vars);
    for (int i = 0; i < n; i++) {
        x0.segment<3>(3 * i) = ctrl_pts[i].cast<c_float>();
    }
    Eigen::Matrix<c_float, Eigen::Dynamic, 1> y0 = Eigen::Matrix<c_float, Eigen::Dynamic, 1>::Zero(A.rows());
    solver_.setWarmStart(x0, y0);
    
    // Solve
    auto status = solver_.solveProblem();
    
    if (status != OsqpEigen::ErrorExitFlag::NoError) {
        ROS_WARN("[BsplineOptimizer] OSQP failed to find optimal solution, using initial trajectory");
        return true;  // Return true to continue with initial trajectory
    }
    
    // Extract solution
    const auto& solution = solver_.getSolution();
    
    for (int i = 0; i < n; i++) {
        ctrl_pts[i] = solution.segment<3>(3 * i).cast<double>();
    }
    
    ROS_INFO("[BsplineOptimizer] Optimization succeeded");
    return true;
}

void BsplineOptimizer::buildSmoothnessCost(Eigen::SparseMatrix<double>& P, Eigen::VectorXd& q) {
    auto& ctrl_pts = trajectory_.getControlPoints();
    int n = ctrl_pts.size();
    double dt = trajectory_.getDt();
    
    // Minimize acceleration (second derivative of B-spline)
    // For cubic B-spline: a = (P_{i+2} - 2*P_{i+1} + P_i) / dt^2
    // Cost: sum ||a_i||^2 = sum ||P_{i+2} - 2*P_{i+1} + P_i||^2 / dt^4
    
    std::vector<Eigen::Triplet<double>> triplets;
    double w = config_.lambda_smooth / (dt * dt * dt * dt);
    
    for (int i = 0; i < n - 2; i++) {
        // For each dimension x, y, z
        for (int d = 0; d < 3; d++) {
            int idx_i = 3 * i + d;
            int idx_i1 = 3 * (i + 1) + d;
            int idx_i2 = 3 * (i + 2) + d;
            
            // Coefficients: [1, -2, 1]
            // P contribution: [1, -2, 1]^T * [1, -2, 1] = [1, -2, 1; -2, 4, -2; 1, -2, 1]
            triplets.push_back(Eigen::Triplet<double>(idx_i, idx_i, w));
            triplets.push_back(Eigen::Triplet<double>(idx_i, idx_i1, -2 * w));
            triplets.push_back(Eigen::Triplet<double>(idx_i, idx_i2, w));
            
            triplets.push_back(Eigen::Triplet<double>(idx_i1, idx_i, -2 * w));
            triplets.push_back(Eigen::Triplet<double>(idx_i1, idx_i1, 4 * w));
            triplets.push_back(Eigen::Triplet<double>(idx_i1, idx_i2, -2 * w));
            
            triplets.push_back(Eigen::Triplet<double>(idx_i2, idx_i, w));
            triplets.push_back(Eigen::Triplet<double>(idx_i2, idx_i1, -2 * w));
            triplets.push_back(Eigen::Triplet<double>(idx_i2, idx_i2, w));
        }
    }
    
    // Add small regularization for numerical stability
    for (int i = 0; i < 3 * n; i++) {
        triplets.push_back(Eigen::Triplet<double>(i, i, 1e-6));
    }
    
    P.resize(3 * n, 3 * n);
    P.setFromTriplets(triplets.begin(), triplets.end());
    
    q = Eigen::VectorXd::Zero(3 * n);
}

void BsplineOptimizer::buildConstraints(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l, Eigen::VectorXd& u) {
    auto& ctrl_pts = trajectory_.getControlPoints();
    int n = ctrl_pts.size();
    double dt = trajectory_.getDt();
    
    std::vector<Eigen::Triplet<double>> triplets;
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    int row = 0;
    
    // 1. Boundary constraints: fix first and last 2 control points
    for (int i = 0; i < 2; i++) {
        for (int d = 0; d < 3; d++) {
            triplets.push_back(Eigen::Triplet<double>(row, 3 * i + d, 1.0));
            lower_bounds.push_back(ctrl_pts[i](d));
            upper_bounds.push_back(ctrl_pts[i](d));
            row++;
        }
    }
    
    for (int i = n - 2; i < n; i++) {
        for (int d = 0; d < 3; d++) {
            triplets.push_back(Eigen::Triplet<double>(row, 3 * i + d, 1.0));
            lower_bounds.push_back(ctrl_pts[i](d));
            upper_bounds.push_back(ctrl_pts[i](d));
            row++;
        }
    }
    
    // 2. Velocity constraints: |P_{i+1} - P_i| / dt <= v_max
    // This is approximated as box constraints on differences
    double v_bound = config_.max_vel * dt;
    for (int i = 0; i < n - 1; i++) {
        for (int d = 0; d < 3; d++) {
            // P_{i+1,d} - P_{i,d} <= v_bound
            triplets.push_back(Eigen::Triplet<double>(row, 3 * (i + 1) + d, 1.0));
            triplets.push_back(Eigen::Triplet<double>(row, 3 * i + d, -1.0));
            lower_bounds.push_back(-v_bound);
            upper_bounds.push_back(v_bound);
            row++;
        }
    }
    
    // 3. Acceleration constraints: |P_{i+2} - 2*P_{i+1} + P_i| / dt^2 <= a_max
    double a_bound = config_.max_acc * dt * dt;
    for (int i = 0; i < n - 2; i++) {
        for (int d = 0; d < 3; d++) {
            // P_{i+2,d} - 2*P_{i+1,d} + P_{i,d} in [-a_bound, a_bound]
            triplets.push_back(Eigen::Triplet<double>(row, 3 * i + d, 1.0));
            triplets.push_back(Eigen::Triplet<double>(row, 3 * (i + 1) + d, -2.0));
            triplets.push_back(Eigen::Triplet<double>(row, 3 * (i + 2) + d, 1.0));
            lower_bounds.push_back(-a_bound);
            upper_bounds.push_back(a_bound);
            row++;
        }
    }
    
    // 4. Corridor constraints (if available)
    // Each control point should be inside its corresponding corridor
    if (!corridors_.empty()) {
        // Map control points to corridors
        // Control points 0,1 are boundary, 2 to n-3 are inner points
        // Map inner control point i to corridor floor((i-2) * num_corridors / (n-4))
        int num_inner = n - 4;  // Excluding 2 at start and 2 at end
        int num_corridors = corridors_.size();
        
        for (int i = 2; i < n - 2; i++) {
            int corridor_idx = std::min(num_corridors - 1, 
                                       (i - 2) * num_corridors / std::max(1, num_inner));
            
            const auto& corridor = corridors_[corridor_idx];
            int num_planes = corridor.A.rows();
            
            for (int j = 0; j < num_planes; j++) {
                // A(j,:) * P_i <= b(j)
                for (int d = 0; d < 3; d++) {
                    if (d < corridor.A.cols()) {
                        triplets.push_back(Eigen::Triplet<double>(row, 3 * i + d, corridor.A(j, d)));
                    }
                }
                lower_bounds.push_back(-OsqpEigen::INFTY);
                upper_bounds.push_back(corridor.b(j));
                row++;
            }
        }
    }
    
    // Build sparse matrix
    A.resize(row, 3 * n);
    A.setFromTriplets(triplets.begin(), triplets.end());
    
    l = Eigen::Map<Eigen::VectorXd>(lower_bounds.data(), lower_bounds.size());
    u = Eigen::Map<Eigen::VectorXd>(upper_bounds.data(), upper_bounds.size());
    
    ROS_INFO("[BsplineOptimizer] Built %d constraints for %d control points", row, n);
}

}  // namespace trajectory_planner
