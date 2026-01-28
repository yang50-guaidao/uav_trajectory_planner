#include "trajectory_planner/plan_env/grid_map.hpp"
#include <pcl/filters/voxel_grid.h>

namespace trajectory_planner {

void GridMap::init(ros::NodeHandle& nh) {
    // Load parameters
    nh.param("grid_map/resolution", config_.resolution, 0.1);
    
    std::vector<double> origin, map_size;
    nh.param("grid_map/origin", origin, std::vector<double>{-15.0, -15.0, 0.0});
    nh.param("grid_map/size", map_size, std::vector<double>{30.0, 30.0, 3.0});
    nh.param("grid_map/inflate_radius", config_.inflate_radius, 0.3);
    
    config_.origin = Eigen::Vector3d(origin[0], origin[1], origin[2]);
    config_.map_size = Eigen::Vector3d(map_size[0], map_size[1], map_size[2]);
    
    initMap(config_);
    
    // Subscribe to point cloud
    cloud_sub_ = nh.subscribe("/structure_map/global_cloud", 1, &GridMap::cloudCallback, this);
    
    ROS_INFO("[GridMap] Initialized with resolution=%.2f, size=[%.1f, %.1f, %.1f]",
             config_.resolution, config_.map_size.x(), config_.map_size.y(), config_.map_size.z());
}

void GridMap::initMap(const Config& config) {
    config_ = config;
    
    // Calculate grid dimensions
    grid_size_ = (config_.map_size / config_.resolution).cast<int>();
    
    // Initialize occupancy grid
    int total_size = grid_size_.x() * grid_size_.y() * grid_size_.z();
    occupancy_.resize(total_size, 0);
    
    ROS_INFO("[GridMap] Grid size: [%d, %d, %d], total cells: %d",
             grid_size_.x(), grid_size_.y(), grid_size_.z(), total_size);
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->empty()) {
        ROS_WARN("[GridMap] Received empty point cloud");
        return;
    }
    
    // Clear previous data
    std::fill(occupancy_.begin(), occupancy_.end(), 0);
    obstacle_points_.clear();
    
    // Store obstacle points and mark occupancy
    for (const auto& pt : cloud->points) {
        Eigen::Vector3d pos(pt.x, pt.y, pt.z);
        
        // Store for decomp_util
        obstacle_points_.push_back(pos);
        
        // Mark in occupancy grid
        Eigen::Vector3i idx = posToIndex(pos);
        if (isInMap(idx)) {
            int linear_idx = indexToLinear(idx);
            occupancy_[linear_idx] = 1;
        }
    }
    
    // Inflate obstacles
    inflateObstacles();
    
    map_ready_ = true;
    ROS_INFO_ONCE("[GridMap] Map updated with %zu obstacle points", obstacle_points_.size());
}

void GridMap::inflateObstacles() {
    if (config_.inflate_radius <= 0) return;
    
    int inflate_cells = static_cast<int>(std::ceil(config_.inflate_radius / config_.resolution));
    
    // Create a copy of current occupancy
    std::vector<int8_t> original_occupancy = occupancy_;
    
    // Inflate each occupied cell
    for (int x = 0; x < grid_size_.x(); x++) {
        for (int y = 0; y < grid_size_.y(); y++) {
            for (int z = 0; z < grid_size_.z(); z++) {
                Eigen::Vector3i idx(x, y, z);
                int linear_idx = indexToLinear(idx);
                
                if (original_occupancy[linear_idx] == 1) {
                    // Inflate around this cell
                    for (int dx = -inflate_cells; dx <= inflate_cells; dx++) {
                        for (int dy = -inflate_cells; dy <= inflate_cells; dy++) {
                            for (int dz = -inflate_cells; dz <= inflate_cells; dz++) {
                                Eigen::Vector3i neighbor(x + dx, y + dy, z + dz);
                                if (isInMap(neighbor)) {
                                    double dist = std::sqrt(dx*dx + dy*dy + dz*dz) * config_.resolution;
                                    if (dist <= config_.inflate_radius) {
                                        occupancy_[indexToLinear(neighbor)] = 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

bool GridMap::isOccupied(const Eigen::Vector3d& pos) const {
    if (!map_ready_) return true;  // Conservative: unknown is occupied
    
    Eigen::Vector3i idx = posToIndex(pos);
    if (!isInMap(idx)) return true;  // Outside map is occupied
    
    return occupancy_[indexToLinear(idx)] == 1;
}

bool GridMap::isInMap(const Eigen::Vector3d& pos) const {
    Eigen::Vector3d rel_pos = pos - config_.origin;
    return rel_pos.x() >= 0 && rel_pos.x() < config_.map_size.x() &&
           rel_pos.y() >= 0 && rel_pos.y() < config_.map_size.y() &&
           rel_pos.z() >= 0 && rel_pos.z() < config_.map_size.z();
}

std::vector<Eigen::Vector3d> GridMap::getObstaclePoints() const {
    return obstacle_points_;
}

void GridMap::getObstaclePointsVec(std::vector<Eigen::Vector3f>& obs_pts) const {
    obs_pts.clear();
    obs_pts.reserve(obstacle_points_.size());
    for (const auto& pt : obstacle_points_) {
        obs_pts.push_back(pt.cast<float>());
    }
}

double GridMap::getDistance(const Eigen::Vector3d& pos) const {
    if (!map_ready_) return 0.0;
    
    // Simple: find minimum distance to any obstacle point
    // For better performance, use KD-tree or compute distance field
    double min_dist = std::numeric_limits<double>::max();
    
    for (const auto& obs : obstacle_points_) {
        double dist = (pos - obs).norm();
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    
    return min_dist;
}

Eigen::Vector3i GridMap::posToIndex(const Eigen::Vector3d& pos) const {
    Eigen::Vector3d rel_pos = pos - config_.origin;
    return (rel_pos / config_.resolution).cast<int>();
}

Eigen::Vector3d GridMap::indexToPos(const Eigen::Vector3i& idx) const {
    return config_.origin + (idx.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * config_.resolution;
}

int GridMap::indexToLinear(const Eigen::Vector3i& idx) const {
    return idx.x() + idx.y() * grid_size_.x() + idx.z() * grid_size_.x() * grid_size_.y();
}

bool GridMap::isInMap(const Eigen::Vector3i& idx) const {
    return idx.x() >= 0 && idx.x() < grid_size_.x() &&
           idx.y() >= 0 && idx.y() < grid_size_.y() &&
           idx.z() >= 0 && idx.z() < grid_size_.z();
}

}  // namespace trajectory_planner
