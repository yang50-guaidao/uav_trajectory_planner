#include <ros/ros.h>
#include "trajectory_planner/plan_manage/planner_manager.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_planner_node");
    ros::NodeHandle nh("~");

    trajectory_planner::PlannerManager planner;
    planner.init(nh);

    ros::spin();
    return 0;
}
