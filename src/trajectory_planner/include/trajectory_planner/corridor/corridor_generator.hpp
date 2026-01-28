#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_utils/data_ros_utils.h>

namespace trajectory_planner {

// Polyhedron representation: Ax <= b
struct Polyhedron {
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::Vector3d center;  // A point inside the polyhedron
};

class CorridorGenerator {
public:
    struct Config {
        double robot_radius = 0.3;
        Eigen::Vector3d local_bbox = Eigen::Vector3d(2.0, 2.0, 1.0);
        Eigen::Vector3d global_bbox_min = Eigen::Vector3d(-15.0, -15.0, 0.0);
        Eigen::Vector3d global_bbox_max = Eigen::Vector3d(15.0, 15.0, 3.0);
    };

    CorridorGenerator() = default;
    ~CorridorGenerator() = default;

    void init(ros::NodeHandle& nh);

    // Generate corridors along path
    bool generateCorridors(const std::vector<Eigen::Vector3d>& path,
                          const std::vector<Eigen::Vector3f>& obstacles);

    // Get generated corridors as Ax <= b constraints
    std::vector<Polyhedron> getCorridors() const { return corridors_; }

    // Get decomp_util polyhedrons (for visualization)
    vec_E<Polyhedron3D> getDecompPolyhedrons() const { return decomp_polys_; }

    // Get ROS message for visualization
    decomp_ros_msgs::PolyhedronArray getCorridorsMsg() const;
    
    // Get ellipsoids for visualization
    vec_E<Ellipsoid3D> getEllipsoids() const { return ellipsoids_; }

    // Check if a point is inside a corridor
    bool isInsideCorridor(int corridor_idx, const Eigen::Vector3d& pt) const;

private:
    Config config_;
    std::vector<Polyhedron> corridors_;
    vec_E<Polyhedron3D> decomp_polys_;  // Original decomp_util polyhedrons
    vec_E<Ellipsoid3D> ellipsoids_;     // Ellipsoids for visualization
};

}  // namespace trajectory_planner
