#pragma once

#include <Eigen/Dense>
#include <vector>

namespace trajectory_planner {

class UniformBspline {
public:
    // Trajectory point for sampling
    struct TrajectoryPoint {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        double time;
    };

    UniformBspline() = default;
    ~UniformBspline() = default;

    // Set control points and time interval
    void setControlPoints(const std::vector<Eigen::Vector3d>& pts, double dt);

    // Evaluate at time t
    Eigen::Vector3d getPosition(double t) const;
    Eigen::Vector3d getVelocity(double t) const;
    Eigen::Vector3d getAcceleration(double t) const;

    // Get total duration
    double getTotalTime() const;

    // Sample trajectory at given frequency (for control)
    std::vector<TrajectoryPoint> sampleTrajectory(double sample_freq) const;

    // Get control points
    std::vector<Eigen::Vector3d>& getControlPoints() { return control_points_; }
    const std::vector<Eigen::Vector3d>& getControlPoints() const { return control_points_; }
    double getDt() const { return dt_; }

private:
    std::vector<Eigen::Vector3d> control_points_;
    double dt_ = 0.1;
    int degree_ = 3;

    static Eigen::Matrix4d M_cubic_;
    static bool matrix_initialized_;
    static void initBasisMatrix();
};

}  // namespace trajectory_planner
