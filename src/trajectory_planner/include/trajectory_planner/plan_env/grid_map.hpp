#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace trajectory_planner {

class GridMap {
public:
    struct Config {
        double resolution = 0.1;
        Eigen::Vector3d origin = Eigen::Vector3d(-15.0, -15.0, 0.0);
        Eigen::Vector3d map_size = Eigen::Vector3d(30.0, 30.0, 3.0);
        double inflate_radius = 0.3;
    };

    GridMap() = default;
    ~GridMap() = default;

    // Initialize with ROS node handle
    void init(ros::NodeHandle& nh);

    // Initialize with config
    void initMap(const Config& config);

    // Check if position is occupied (collision)
    bool isOccupied(const Eigen::Vector3d& pos) const;

    // Check if position is within map bounds
    bool isInMap(const Eigen::Vector3d& pos) const;

    // Get obstacle points for decomp_util
    std::vector<Eigen::Vector3d> getObstaclePoints() const;

    // Get obstacle points as vec_Vec3f for decomp_util
    void getObstaclePointsVec(std::vector<Eigen::Vector3f>& obs_pts) const;

    // Check if map is ready
    bool isMapReady() const { return map_ready_; }

    // Getters
    double getResolution() const { return config_.resolution; }
    Eigen::Vector3d getOrigin() const { return config_.origin; }
    Eigen::Vector3d getMapSize() const { return config_.map_size; }
    Eigen::Vector3i getGridSize() const { return grid_size_; }

    // Get distance to nearest obstacle (simple version)
    double getDistance(const Eigen::Vector3d& pos) const;

private:
    // Point cloud callback
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Convert position to grid index
    Eigen::Vector3i posToIndex(const Eigen::Vector3d& pos) const;

    // Convert grid index to position
    Eigen::Vector3d indexToPos(const Eigen::Vector3i& idx) const;

    // Convert 3D index to linear index
    int indexToLinear(const Eigen::Vector3i& idx) const;

    // Check if index is within bounds
    bool isInMap(const Eigen::Vector3i& idx) const;

    // Inflate obstacles
    void inflateObstacles();

    Config config_;
    Eigen::Vector3i grid_size_;
    std::vector<int8_t> occupancy_;  // 0: free, 1: occupied
    std::vector<Eigen::Vector3d> obstacle_points_;  // Store original obstacle points

    ros::Subscriber cloud_sub_;
    bool map_ready_ = false;
};

}  // namespace trajectory_planner
