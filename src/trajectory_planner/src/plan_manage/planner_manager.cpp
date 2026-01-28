#include "trajectory_planner/plan_manage/planner_manager.hpp"
#include <decomp_ros_utils/data_ros_utils.h>

namespace trajectory_planner {

void PlannerManager::init(ros::NodeHandle& nh) {
    nh_ = nh;

    // Initialize modules
    grid_map_ = std::make_shared<GridMap>();
    path_finder_ = std::make_shared<KinodynamicAstar>();
    corridor_gen_ = std::make_shared<CorridorGenerator>();
    optimizer_ = std::make_shared<BsplineOptimizer>();

    grid_map_->init(nh);
    path_finder_->init(nh);
    path_finder_->setGridMap(grid_map_);
    corridor_gen_->init(nh);
    optimizer_->init(nh);

    // Load start position
    std::vector<double> start;
    nh.param("planner/start_pos", start, std::vector<double>{0.0, 0.0, 1.0});
    start_pos_ = Eigen::Vector3d(start[0], start[1], start[2]);

    // Publishers (latched=true so RViz can receive even if subscribed later)
    path_pub_ = nh.advertise<visualization_msgs::Marker>("planning/path", 1, true);
    corridor_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("planning/corridors", 1, true);
    ellipsoid_pub_ = nh.advertise<decomp_ros_msgs::EllipsoidArray>("planning/ellipsoids", 1, true);
    traj_pub_ = nh.advertise<visualization_msgs::Marker>("planning/trajectory", 1, true);
    ctrl_pts_pub_ = nh.advertise<visualization_msgs::Marker>("planning/control_points", 1, true);

    // Subscriber
    goal_sub_ = nh.subscribe("planning/goal", 1, &PlannerManager::goalCallback, this);

    // Timer for planning check
    planning_timer_ = nh.createTimer(ros::Duration(0.1), &PlannerManager::planningTimerCallback, this);

    ROS_INFO("[PlannerManager] Initialized, waiting for goal...");
    ROS_INFO("[PlannerManager] Start position: [%.2f, %.2f, %.2f]", 
             start_pos_.x(), start_pos_.y(), start_pos_.z());
}

void PlannerManager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_pos_.x() = msg->pose.position.x;
    goal_pos_.y() = msg->pose.position.y;
    goal_pos_.z() = msg->pose.position.z;
    
    // If z is 0 (from 2D Nav Goal), set to default height
    if (std::abs(goal_pos_.z()) < 0.1) {
        goal_pos_.z() = start_pos_.z();
    }

    has_goal_ = true;
    state_ = PlannerState::PLANNING;
    
    ROS_INFO("[PlannerManager] New goal received: [%.2f, %.2f, %.2f]",
             goal_pos_.x(), goal_pos_.y(), goal_pos_.z());
}

void PlannerManager::planningTimerCallback(const ros::TimerEvent& event) {
    if (state_ != PlannerState::PLANNING) return;
    
    if (!grid_map_->isMapReady()) {
        ROS_WARN_THROTTLE(1.0, "[PlannerManager] Waiting for map...");
        return;
    }

    if (runPlanning()) {
        state_ = PlannerState::READY;
        ROS_INFO("[PlannerManager] Planning succeeded, state -> READY");
    } else {
        state_ = PlannerState::IDLE;
        has_goal_ = false;
        ROS_WARN("[PlannerManager] Planning failed, state -> IDLE");
    }
}

bool PlannerManager::runPlanning() {
    ros::Time start_time = ros::Time::now();

    // Step 1: Path search
    ROS_INFO("[PlannerManager] Running Kinodynamic A*...");
    if (!path_finder_->search(start_pos_, start_vel_, goal_pos_, goal_vel_)) {
        ROS_WARN("[PlannerManager] Path search failed!");
        return false;
    }
    
    current_path_ = path_finder_->getPath();
    double search_time = (ros::Time::now() - start_time).toSec() * 1000;
    ROS_INFO("[PlannerManager] Path search completed: %zu points, %.2f ms",
             current_path_.size(), search_time);

    // Publish path visualization
    publishPathVisualization();

    // Step 2: Corridor generation
    ros::Time corridor_start = ros::Time::now();
    std::vector<Eigen::Vector3f> obstacles;
    grid_map_->getObstaclePointsVec(obstacles);
    
    if (!corridor_gen_->generateCorridors(current_path_, obstacles)) {
        ROS_WARN("[PlannerManager] Corridor generation failed!");
        // Continue anyway, optimizer will handle empty corridors
    }
    
    double corridor_time = (ros::Time::now() - corridor_start).toSec() * 1000;
    ROS_INFO("[PlannerManager] Corridor generation: %zu corridors, %.2f ms",
             corridor_gen_->getCorridors().size(), corridor_time);
    
    // Publish corridor visualization
    publishCorridorVisualization();
    
    // Step 3: B-spline optimization (placeholder for Phase 5)
    double path_length = 0;
    for (size_t i = 1; i < current_path_.size(); i++) {
        path_length += (current_path_[i] - current_path_[i-1]).norm();
    }
    double total_time = path_length / 2.0 * 1.5;  // Conservative time
    
    optimizer_->setInitialPath(current_path_, total_time);
    optimizer_->setCorridors(corridor_gen_->getCorridors());
    optimizer_->optimize();
    
    current_traj_ = optimizer_->getTrajectory();
    
    // Publish trajectory visualization
    publishTrajectoryVisualization();

    double total_plan_time = (ros::Time::now() - start_time).toSec() * 1000;
    ROS_INFO("[PlannerManager] Total planning time: %.2f ms", total_plan_time);

    return true;
}

void PlannerManager::publishPathVisualization() {
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.1;
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;
    path_marker.pose.orientation.w = 1.0;

    for (const auto& pt : current_path_) {
        geometry_msgs::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        path_marker.points.push_back(p);
    }

    path_pub_.publish(path_marker);
    
    // Also publish path points as spheres
    visualization_msgs::Marker pts_marker = path_marker;
    pts_marker.id = 1;
    pts_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    pts_marker.scale.x = 0.15;
    pts_marker.scale.y = 0.15;
    pts_marker.scale.z = 0.15;
    pts_marker.color.r = 0.0;
    pts_marker.color.g = 0.8;
    pts_marker.color.b = 0.0;
    
    path_pub_.publish(pts_marker);
}

void PlannerManager::publishCorridorVisualization() {
    // Publish polyhedron corridors (for decomp_rviz_plugins)
    auto poly_msg = corridor_gen_->getCorridorsMsg();
    corridor_pub_.publish(poly_msg);
    
    // Publish ellipsoids
    auto ellipsoids = corridor_gen_->getEllipsoids();
    auto ellipsoid_msg = DecompROS::ellipsoid_array_to_ros<3>(ellipsoids);
    ellipsoid_msg.header.frame_id = "world";
    ellipsoid_msg.header.stamp = ros::Time::now();
    ellipsoid_pub_.publish(ellipsoid_msg);
    
    ROS_INFO("[PlannerManager] Published %zu polyhedrons, %zu ellipsoids", 
             poly_msg.polyhedrons.size(), ellipsoids.size());
}

void PlannerManager::publishTrajectoryVisualization() {
    if (current_traj_.getTotalTime() <= 0) return;

    // Trajectory line
    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id = "world";
    traj_marker.header.stamp = ros::Time::now();
    traj_marker.ns = "trajectory";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.scale.x = 0.08;
    traj_marker.color.r = 1.0;
    traj_marker.color.g = 0.0;
    traj_marker.color.b = 0.0;
    traj_marker.color.a = 1.0;
    traj_marker.pose.orientation.w = 1.0;

    double total_time = current_traj_.getTotalTime();
    for (double t = 0; t <= total_time; t += 0.05) {
        Eigen::Vector3d pos = current_traj_.getPosition(t);
        geometry_msgs::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = pos.z();
        traj_marker.points.push_back(p);
    }

    traj_pub_.publish(traj_marker);

    // Control points
    visualization_msgs::Marker ctrl_marker;
    ctrl_marker.header = traj_marker.header;
    ctrl_marker.ns = "control_points";
    ctrl_marker.id = 0;
    ctrl_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    ctrl_marker.action = visualization_msgs::Marker::ADD;
    ctrl_marker.scale.x = 0.12;
    ctrl_marker.scale.y = 0.12;
    ctrl_marker.scale.z = 0.12;
    ctrl_marker.color.r = 0.0;
    ctrl_marker.color.g = 0.0;
    ctrl_marker.color.b = 1.0;
    ctrl_marker.color.a = 1.0;
    ctrl_marker.pose.orientation.w = 1.0;

    for (const auto& cp : current_traj_.getControlPoints()) {
        geometry_msgs::Point p;
        p.x = cp.x();
        p.y = cp.y();
        p.z = cp.z();
        ctrl_marker.points.push_back(p);
    }

    ctrl_pts_pub_.publish(ctrl_marker);
}

void PlannerManager::setGoal(const Eigen::Vector3d& goal) {
    goal_pos_ = goal;
    has_goal_ = true;
    state_ = PlannerState::PLANNING;
}

}  // namespace trajectory_planner
