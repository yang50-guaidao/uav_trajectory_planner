#include "trajectory_planner/bspline/uniform_bspline.hpp"

namespace trajectory_planner {

Eigen::Matrix4d UniformBspline::M_cubic_ = Eigen::Matrix4d::Zero();
bool UniformBspline::matrix_initialized_ = false;

void UniformBspline::initBasisMatrix() {
    if (matrix_initialized_) return;
    
    M_cubic_ << -1,  3, -3,  1,
                 3, -6,  3,  0,
                -3,  0,  3,  0,
                 1,  4,  1,  0;
    M_cubic_ /= 6.0;
    
    matrix_initialized_ = true;
}

void UniformBspline::setControlPoints(const std::vector<Eigen::Vector3d>& pts, double dt) {
    control_points_ = pts;
    dt_ = dt;
    initBasisMatrix();
}

double UniformBspline::getTotalTime() const {
    if (control_points_.size() < 4) return 0.0;
    return (control_points_.size() - 3) * dt_;
}

Eigen::Vector3d UniformBspline::getPosition(double t) const {
    if (control_points_.size() < 4) return Eigen::Vector3d::Zero();
    
    double total_time = getTotalTime();
    t = std::max(0.0, std::min(t, total_time - 1e-6));
    
    int seg_idx = static_cast<int>(t / dt_);
    seg_idx = std::min(seg_idx, static_cast<int>(control_points_.size()) - 4);
    
    double u = (t - seg_idx * dt_) / dt_;
    
    Eigen::Vector4d U;
    U << u*u*u, u*u, u, 1;
    
    Eigen::Matrix<double, 3, 4> local_ctrl;
    for (int i = 0; i < 4; i++) {
        local_ctrl.col(i) = control_points_[seg_idx + i];
    }
    
    return local_ctrl * M_cubic_.transpose() * U;
}

Eigen::Vector3d UniformBspline::getVelocity(double t) const {
    if (control_points_.size() < 4) return Eigen::Vector3d::Zero();
    
    double total_time = getTotalTime();
    t = std::max(0.0, std::min(t, total_time - 1e-6));
    
    int seg_idx = static_cast<int>(t / dt_);
    seg_idx = std::min(seg_idx, static_cast<int>(control_points_.size()) - 4);
    
    double u = (t - seg_idx * dt_) / dt_;
    
    Eigen::Vector4d dU;
    dU << 3*u*u, 2*u, 1, 0;
    
    Eigen::Matrix<double, 3, 4> local_ctrl;
    for (int i = 0; i < 4; i++) {
        local_ctrl.col(i) = control_points_[seg_idx + i];
    }
    
    return local_ctrl * M_cubic_.transpose() * dU / dt_;
}

Eigen::Vector3d UniformBspline::getAcceleration(double t) const {
    if (control_points_.size() < 4) return Eigen::Vector3d::Zero();
    
    double total_time = getTotalTime();
    t = std::max(0.0, std::min(t, total_time - 1e-6));
    
    int seg_idx = static_cast<int>(t / dt_);
    seg_idx = std::min(seg_idx, static_cast<int>(control_points_.size()) - 4);
    
    double u = (t - seg_idx * dt_) / dt_;
    
    Eigen::Vector4d ddU;
    ddU << 6*u, 2, 0, 0;
    
    Eigen::Matrix<double, 3, 4> local_ctrl;
    for (int i = 0; i < 4; i++) {
        local_ctrl.col(i) = control_points_[seg_idx + i];
    }
    
    return local_ctrl * M_cubic_.transpose() * ddU / (dt_ * dt_);
}

std::vector<UniformBspline::TrajectoryPoint> UniformBspline::sampleTrajectory(double sample_freq) const {
    std::vector<TrajectoryPoint> traj;
    
    double total_time = getTotalTime();
    if (total_time <= 0) return traj;
    
    double dt = 1.0 / sample_freq;
    
    for (double t = 0; t <= total_time; t += dt) {
        TrajectoryPoint pt;
        pt.time = t;
        pt.position = getPosition(t);
        pt.velocity = getVelocity(t);
        pt.acceleration = getAcceleration(t);
        traj.push_back(pt);
    }
    
    return traj;
}

}  // namespace trajectory_planner
