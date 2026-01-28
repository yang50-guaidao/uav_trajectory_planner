#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>
#include "trajectory_planner/plan_env/grid_map.hpp"

namespace trajectory_planner {

class KinodynamicAstar {
public:
    struct Config {
        double max_vel = 3.0;           // m/s
        double max_acc = 6.0;           // m/s^2
        double resolution = 0.5;         // spatial resolution for state discretization
        double time_resolution = 0.1;    // time step for motion primitives
        double lambda_heuristic = 1.0;   // heuristic weight
        int max_iterations = 10000;
        double goal_tolerance = 0.5;     // distance to consider goal reached
    };

    KinodynamicAstar() = default;
    ~KinodynamicAstar() = default;

    // Initialize with parameters
    void init(ros::NodeHandle& nh);

    // Set grid map for collision checking
    void setGridMap(std::shared_ptr<GridMap> grid_map);

    // Main search function
    bool search(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                const Eigen::Vector3d& goal_pos, const Eigen::Vector3d& goal_vel);

    // Get the result path
    std::vector<Eigen::Vector3d> getPath() const { return path_; }

    // Get path with velocity info
    void getPathWithVelocity(std::vector<Eigen::Vector3d>& positions,
                             std::vector<Eigen::Vector3d>& velocities) const;

    // Reset search
    void reset();

private:
    struct Node {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        double g_cost;
        double f_cost;
        int parent_idx;
        Eigen::Vector3d input;  // acceleration input to reach this state
        double duration;        // time duration
        
        bool operator>(const Node& other) const {
            return f_cost > other.f_cost;
        }
    };

    // Compute heuristic cost
    double computeHeuristic(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                           const Eigen::Vector3d& goal_pos, const Eigen::Vector3d& goal_vel) const;

    // State to hash key for closed set
    int64_t stateToKey(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) const;

    // Check collision along trajectory segment
    bool checkCollisionFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) const;

    // Check if velocity is within limits
    bool isVelocityValid(const Eigen::Vector3d& vel) const;

    // Try analytic expansion to goal
    bool tryAnalyticExpansion(const Node& current, const Eigen::Vector3d& goal_pos,
                              const Eigen::Vector3d& goal_vel);

    // Retrieve path from search result
    void retrievePath(int goal_idx);

    Config config_;
    std::shared_ptr<GridMap> grid_map_;

    // Discrete control inputs
    std::vector<Eigen::Vector3d> control_inputs_;
    std::vector<double> time_durations_;

    // Search data
    std::vector<Node> all_nodes_;
    std::unordered_map<int64_t, int> closed_set_;  // state key -> node index

    // Result
    std::vector<Eigen::Vector3d> path_;
    std::vector<Eigen::Vector3d> velocities_;
};

}  // namespace trajectory_planner
