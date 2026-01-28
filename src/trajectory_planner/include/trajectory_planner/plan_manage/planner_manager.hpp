#pragma once

#include <ros/ros.h>
#include <memory>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <decomp_ros_msgs/EllipsoidArray.h>
#include <trajectory_planner/BsplineTrajectory.h>

#include "trajectory_planner/plan_env/grid_map.hpp"
#include "trajectory_planner/path_searching/kinodynamic_astar.hpp"
#include "trajectory_planner/corridor/corridor_generator.hpp"
#include "trajectory_planner/bspline/bspline_optimizer.hpp"

namespace trajectory_planner {

enum class PlannerState {
    IDLE,
    PLANNING,
    READY
};

class PlannerManager {
public:
    PlannerManager() = default;
    ~PlannerManager() = default;

    void init(ros::NodeHandle& nh);

    // Set goal and trigger planning
    void setGoal(const Eigen::Vector3d& goal);

private:
    // Callbacks
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void planningTimerCallback(const ros::TimerEvent& event);

    // Planning pipeline
    bool runPlanning();

    // Visualization
    void publishPathVisualization();
    void publishCorridorVisualization();
    void publishTrajectoryVisualization();
    
    // Trajectory publishing for execution
    void publishBsplineTrajectory();

    ros::NodeHandle nh_;
    
    // Modules
    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<KinodynamicAstar> path_finder_;
    std::shared_ptr<CorridorGenerator> corridor_gen_;
    std::shared_ptr<BsplineOptimizer> optimizer_;

    // ROS
    ros::Subscriber goal_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher path_pub_;
    ros::Publisher corridor_pub_;
    ros::Publisher ellipsoid_pub_;
    ros::Publisher traj_pub_;
    ros::Publisher ctrl_pts_pub_;
    ros::Publisher bspline_traj_pub_;  // For traj_server
    ros::Timer planning_timer_;

    // State
    PlannerState state_ = PlannerState::IDLE;
    Eigen::Vector3d start_pos_ = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d start_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_pos_;
    Eigen::Vector3d goal_vel_ = Eigen::Vector3d::Zero();
    bool has_goal_ = false;
    bool has_odom_ = false;
    bool use_odom_as_start_ = true;  // Use current position as start

    // Results
    std::vector<Eigen::Vector3d> current_path_;
    UniformBspline current_traj_;
};

}  // namespace trajectory_planner
