#include "trajectory_planner/corridor/corridor_generator.hpp"

namespace trajectory_planner {

void CorridorGenerator::init(ros::NodeHandle& nh) {
    std::vector<double> bbox, global_min, global_max;
    nh.param("corridor/robot_radius", config_.robot_radius, 0.3);
    nh.param("corridor/local_bbox", bbox, std::vector<double>{2.0, 2.0, 1.0});
    nh.param("grid_map/origin", global_min, std::vector<double>{-15.0, -15.0, 0.0});
    
    std::vector<double> map_size;
    nh.param("grid_map/size", map_size, std::vector<double>{30.0, 30.0, 3.0});
    
    config_.local_bbox = Eigen::Vector3d(bbox[0], bbox[1], bbox[2]);
    config_.global_bbox_min = Eigen::Vector3d(global_min[0], global_min[1], global_min[2]);
    config_.global_bbox_max = config_.global_bbox_min + Eigen::Vector3d(map_size[0], map_size[1], map_size[2]);
    
    ROS_INFO("[CorridorGenerator] Initialized with local_bbox=[%.1f, %.1f, %.1f]",
             config_.local_bbox.x(), config_.local_bbox.y(), config_.local_bbox.z());
}

bool CorridorGenerator::generateCorridors(const std::vector<Eigen::Vector3d>& path,
                                          const std::vector<Eigen::Vector3f>& obstacles) {
    corridors_.clear();
    decomp_polys_.clear();
    ellipsoids_.clear();

    if (path.size() < 2) {
        ROS_WARN("[CorridorGenerator] Path too short for corridor generation");
        return false;
    }

    // Convert path to decomp_util format (vec_Vec3f)
    vec_Vec3f decomp_path;
    decomp_path.reserve(path.size());
    for (const auto& pt : path) {
        decomp_path.push_back(pt.cast<decimal_t>());
    }

    // Convert obstacles to decomp_util format
    vec_Vec3f obs_pts;
    obs_pts.reserve(obstacles.size());
    for (const auto& pt : obstacles) {
        obs_pts.push_back(pt.cast<decimal_t>());
    }

    ROS_INFO("[CorridorGenerator] Generating corridors for %zu path segments with %zu obstacles",
             path.size() - 1, obstacles.size());

    // Create EllipsoidDecomp WITHOUT global bounding box (like official example)
    EllipsoidDecomp3D decomp;
    
    // Set obstacle points
    decomp.set_obs(obs_pts);
    
    // Set local bounding box (search range around each line segment)
    decomp.set_local_bbox(config_.local_bbox.cast<decimal_t>());
    
    // Dilate (generate corridors)
    decomp.dilate(decomp_path);
    
    // Get results
    decomp_polys_ = decomp.get_polyhedrons();
    ellipsoids_ = decomp.get_ellipsoids();
    
    // Get linear constraints (Ax <= b format)
    auto constraints = decomp.get_constraints();
    
    // Convert to our Polyhedron format
    corridors_.resize(constraints.size());
    for (size_t i = 0; i < constraints.size(); i++) {
        corridors_[i].A = constraints[i].A();
        corridors_[i].b = constraints[i].b();
        // Center point is midpoint of the path segment
        corridors_[i].center = (path[i] + path[i + 1]) / 2.0;
    }
    
    ROS_INFO("[CorridorGenerator] Generated %zu corridors, %zu ellipsoids", 
             corridors_.size(), ellipsoids_.size());
    
    return !corridors_.empty();
}

decomp_ros_msgs::PolyhedronArray CorridorGenerator::getCorridorsMsg() const {
    auto msg = DecompROS::polyhedron_array_to_ros<3>(decomp_polys_);
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    return msg;
}

bool CorridorGenerator::isInsideCorridor(int corridor_idx, const Eigen::Vector3d& pt) const {
    if (corridor_idx < 0 || corridor_idx >= static_cast<int>(decomp_polys_.size())) {
        return false;
    }
    return decomp_polys_[corridor_idx].inside(pt.cast<decimal_t>());
}

}  // namespace trajectory_planner
