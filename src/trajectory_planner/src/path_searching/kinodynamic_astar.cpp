#include "trajectory_planner/path_searching/kinodynamic_astar.hpp"
#include <cmath>

namespace trajectory_planner {

void KinodynamicAstar::init(ros::NodeHandle& nh) {
    // Load parameters
    nh.param("kinodynamic_astar/max_vel", config_.max_vel, 3.0);
    nh.param("kinodynamic_astar/max_acc", config_.max_acc, 6.0);
    nh.param("kinodynamic_astar/resolution", config_.resolution, 0.5);
    nh.param("kinodynamic_astar/time_resolution", config_.time_resolution, 0.1);
    nh.param("kinodynamic_astar/lambda_heuristic", config_.lambda_heuristic, 1.0);
    nh.param("kinodynamic_astar/max_iterations", config_.max_iterations, 10000);
    nh.param("kinodynamic_astar/goal_tolerance", config_.goal_tolerance, 0.5);

    // Initialize control inputs (discretized accelerations)
    std::vector<double> acc_samples = {-config_.max_acc, 0, config_.max_acc};
    
    for (double ax : acc_samples) {
        for (double ay : acc_samples) {
            for (double az : acc_samples) {
                // Skip zero acceleration to reduce branching
                if (std::abs(ax) < 1e-6 && std::abs(ay) < 1e-6 && std::abs(az) < 1e-6) {
                    control_inputs_.push_back(Eigen::Vector3d(0, 0, 0));  // Keep one zero
                } else {
                    control_inputs_.push_back(Eigen::Vector3d(ax, ay, az));
                }
            }
        }
    }
    
    // Remove duplicates
    control_inputs_.erase(std::unique(control_inputs_.begin(), control_inputs_.end(),
        [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            return (a - b).norm() < 1e-6;
        }), control_inputs_.end());

    // Time durations for motion primitives (larger steps for faster search)
    time_durations_ = {0.8, 0.4, 0.2};

    ROS_INFO("[KinodynamicAstar] Initialized with max_vel=%.2f, max_acc=%.2f, %zu control inputs",
             config_.max_vel, config_.max_acc, control_inputs_.size());
}

void KinodynamicAstar::setGridMap(std::shared_ptr<GridMap> grid_map) {
    grid_map_ = grid_map;
}

void KinodynamicAstar::reset() {
    all_nodes_.clear();
    closed_set_.clear();
    path_.clear();
    velocities_.clear();
}

bool KinodynamicAstar::search(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                               const Eigen::Vector3d& goal_pos, const Eigen::Vector3d& goal_vel) {
    reset();
    
    if (!grid_map_ || !grid_map_->isMapReady()) {
        ROS_WARN("[KinodynamicAstar] Map not ready!");
        return false;
    }

    // Check start and goal validity
    if (grid_map_->isOccupied(start_pos)) {
        ROS_WARN("[KinodynamicAstar] Start position is in collision!");
        return false;
    }
    
    if (grid_map_->isOccupied(goal_pos)) {
        ROS_WARN("[KinodynamicAstar] Goal position is in collision!");
        return false;
    }

    // Priority queue (min-heap by f_cost)
    auto cmp = [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, decltype(cmp)> open_set(cmp);

    // Initialize start node
    Node start_node;
    start_node.pos = start_pos;
    start_node.vel = start_vel;
    start_node.g_cost = 0.0;
    start_node.f_cost = computeHeuristic(start_pos, start_vel, goal_pos, goal_vel);
    start_node.parent_idx = -1;
    start_node.input = Eigen::Vector3d::Zero();
    start_node.duration = 0.0;

    all_nodes_.push_back(start_node);
    open_set.push({start_node.f_cost, 0});
    closed_set_[stateToKey(start_pos, start_vel)] = 0;

    int iterations = 0;
    ros::Time start_time = ros::Time::now();

    while (!open_set.empty() && iterations < config_.max_iterations) {
        iterations++;

        // Get node with minimum f_cost
        auto [f_cost, current_idx] = open_set.top();
        open_set.pop();

        Node& current = all_nodes_[current_idx];

        // Check if reached goal
        double dist_to_goal = (current.pos - goal_pos).norm();
        double vel_diff = (current.vel - goal_vel).norm();
        
        if (dist_to_goal < config_.goal_tolerance && vel_diff < config_.max_vel * 0.2) {
            // Goal reached
            retrievePath(current_idx);
            
            double search_time = (ros::Time::now() - start_time).toSec() * 1000;
            ROS_INFO("[KinodynamicAstar] Path found! Iterations: %d, Time: %.2f ms, Path points: %zu",
                     iterations, search_time, path_.size());
            return true;
        }

        // Try analytic expansion
        if (tryAnalyticExpansion(current, goal_pos, goal_vel)) {
            double search_time = (ros::Time::now() - start_time).toSec() * 1000;
            ROS_INFO("[KinodynamicAstar] Path found via analytic expansion! Time: %.2f ms, Path points: %zu",
                     search_time, path_.size());
            return true;
        }

        // Expand neighbors using motion primitives
        for (const auto& u : control_inputs_) {
            for (double tau : time_durations_) {
                // Forward propagation using double integrator
                // x_new = x + v*tau + 0.5*u*tau^2
                // v_new = v + u*tau
                Eigen::Vector3d new_pos = current.pos + current.vel * tau + 0.5 * u * tau * tau;
                Eigen::Vector3d new_vel = current.vel + u * tau;

                // Check velocity constraints
                if (!isVelocityValid(new_vel)) continue;

                // Check collision
                if (!checkCollisionFree(current.pos, new_pos)) continue;
                if (grid_map_->isOccupied(new_pos)) continue;

                // Check if already visited
                int64_t state_key = stateToKey(new_pos, new_vel);
                if (closed_set_.find(state_key) != closed_set_.end()) continue;

                // Compute costs
                double move_cost = tau + 0.1 * u.squaredNorm() * tau;  // Time + control effort
                double g_cost = current.g_cost + move_cost;
                double h_cost = computeHeuristic(new_pos, new_vel, goal_pos, goal_vel);
                double f_cost = g_cost + config_.lambda_heuristic * h_cost;

                // Create new node
                Node new_node;
                new_node.pos = new_pos;
                new_node.vel = new_vel;
                new_node.g_cost = g_cost;
                new_node.f_cost = f_cost;
                new_node.parent_idx = current_idx;
                new_node.input = u;
                new_node.duration = tau;

                int new_idx = all_nodes_.size();
                all_nodes_.push_back(new_node);
                closed_set_[state_key] = new_idx;
                open_set.push({f_cost, new_idx});
            }
        }
    }

    ROS_WARN("[KinodynamicAstar] Search failed after %d iterations", iterations);
    return false;
}

double KinodynamicAstar::computeHeuristic(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                                          const Eigen::Vector3d& goal_pos, const Eigen::Vector3d& goal_vel) const {
    // Simple heuristic: estimated time based on distance and velocity difference
    Eigen::Vector3d diff = goal_pos - pos;
    double dist = diff.norm();
    double vel_diff = (goal_vel - vel).norm();
    
    // Estimate time to reach goal
    double time_est = dist / config_.max_vel + vel_diff / config_.max_acc;
    
    return time_est;
}

int64_t KinodynamicAstar::stateToKey(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) const {
    // Discretize position with finer resolution
    int px = static_cast<int>(std::floor(pos.x() / config_.resolution));
    int py = static_cast<int>(std::floor(pos.y() / config_.resolution));
    int pz = static_cast<int>(std::floor(pos.z() / config_.resolution));
    
    // Coarser velocity discretization to allow more exploration
    double vel_res = config_.max_vel / 2.0;  // 2 velocity buckets per axis
    int vx = static_cast<int>(std::floor(vel.x() / vel_res)) + 3;
    int vy = static_cast<int>(std::floor(vel.y() / vel_res)) + 3;
    int vz = static_cast<int>(std::floor(vel.z() / vel_res)) + 3;
    
    // Combine into single key (use only position for now to allow more exploration)
    int64_t key = 0;
    key = (px + 500) * 1000L * 100L + (py + 500) * 100L + (pz + 50);
    
    return key;
}

bool KinodynamicAstar::checkCollisionFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) const {
    double dist = (p2 - p1).norm();
    int num_checks = static_cast<int>(std::ceil(dist / (grid_map_->getResolution() * 0.5))) + 1;
    
    for (int i = 0; i <= num_checks; i++) {
        double t = static_cast<double>(i) / num_checks;
        Eigen::Vector3d pt = p1 + t * (p2 - p1);
        if (grid_map_->isOccupied(pt)) {
            return false;
        }
    }
    return true;
}

bool KinodynamicAstar::isVelocityValid(const Eigen::Vector3d& vel) const {
    return vel.norm() <= config_.max_vel * 1.1;  // Small tolerance
}

bool KinodynamicAstar::tryAnalyticExpansion(const Node& current, const Eigen::Vector3d& goal_pos,
                                            const Eigen::Vector3d& goal_vel) {
    // Simple check: if we can directly connect to goal
    double dist = (current.pos - goal_pos).norm();
    
    if (dist < config_.resolution * 5) {
        if (checkCollisionFree(current.pos, goal_pos)) {
            // Direct connection possible
            path_.clear();
            velocities_.clear();
            
            // Retrieve path from start to current
            std::vector<int> node_indices;
            int idx = all_nodes_.size() - 1;
            while (idx >= 0) {
                node_indices.push_back(idx);
                idx = all_nodes_[idx].parent_idx;
            }
            
            // Reverse to get start to current
            for (auto it = node_indices.rbegin(); it != node_indices.rend(); ++it) {
                path_.push_back(all_nodes_[*it].pos);
                velocities_.push_back(all_nodes_[*it].vel);
            }
            
            // Add goal
            path_.push_back(goal_pos);
            velocities_.push_back(goal_vel);
            
            return true;
        }
    }
    
    return false;
}

void KinodynamicAstar::retrievePath(int goal_idx) {
    path_.clear();
    velocities_.clear();
    
    std::vector<int> node_indices;
    int idx = goal_idx;
    
    while (idx >= 0) {
        node_indices.push_back(idx);
        idx = all_nodes_[idx].parent_idx;
    }
    
    // Reverse to get path from start to goal
    for (auto it = node_indices.rbegin(); it != node_indices.rend(); ++it) {
        path_.push_back(all_nodes_[*it].pos);
        velocities_.push_back(all_nodes_[*it].vel);
    }
}

void KinodynamicAstar::getPathWithVelocity(std::vector<Eigen::Vector3d>& positions,
                                           std::vector<Eigen::Vector3d>& velocities) const {
    positions = path_;
    velocities = velocities_;
}

}  // namespace trajectory_planner
