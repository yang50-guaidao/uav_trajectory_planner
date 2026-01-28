#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include "trajectory_planner/bspline/uniform_bspline.hpp"
#include "trajectory_planner/corridor/corridor_generator.hpp"

namespace trajectory_planner {

class BsplineOptimizer {
public:
    struct Config {
        double lambda_smooth = 10.0;
        double max_vel = 3.0;
        double max_acc = 6.0;
        double control_point_dist = 0.4;
        int max_iterations = 1000;
        double eps_abs = 1e-4;
    };

    BsplineOptimizer() = default;
    ~BsplineOptimizer() = default;

    void init(ros::NodeHandle& nh);

    // Set initial trajectory from path
    void setInitialPath(const std::vector<Eigen::Vector3d>& path, double total_time);

    // Set corridor constraints
    void setCorridors(const std::vector<Polyhedron>& corridors);

    // Run optimization
    bool optimize();

    // Get result
    UniformBspline getTrajectory() const { return trajectory_; }

private:
    // Build QP problem matrices
    void buildSmoothnessCost(Eigen::SparseMatrix<double>& P, Eigen::VectorXd& q);
    void buildConstraints(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l, Eigen::VectorXd& u);

    Config config_;
    UniformBspline trajectory_;
    std::vector<Polyhedron> corridors_;
    
    OsqpEigen::Solver solver_;
};

}  // namespace trajectory_planner
