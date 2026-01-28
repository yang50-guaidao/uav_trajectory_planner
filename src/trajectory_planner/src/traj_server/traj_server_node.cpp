/**
 * @file traj_server_node.cpp
 * @brief Trajectory server that converts B-spline trajectory to position commands
 *        for SO3 controller at 100Hz
 */

#include <ros/ros.h>
#include <Eigen/Dense>
#include <kr_mav_msgs/PositionCommand.h>
#include <trajectory_planner/BsplineTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <topic_tools/MuxSelect.h>

namespace trajectory_planner {

class TrajServer {
public:
    TrajServer(ros::NodeHandle& nh) : nh_(nh), has_traj_(false), executing_(false) {
        // Publishers
        pos_cmd_pub_ = nh_.advertise<kr_mav_msgs::PositionCommand>("position_cmd", 10);
        
        // Subscribers
        traj_sub_ = nh_.subscribe("trajectory", 1, &TrajServer::trajectoryCallback, this);
        odom_sub_ = nh_.subscribe("odom", 10, &TrajServer::odomCallback, this);
        
        // Timer for 100Hz command publishing
        cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &TrajServer::cmdTimerCallback, this);
        
        // Mux select service client
        nh_.param<std::string>("mux_select_service", mux_service_name_, "/quadrotor/cmd_mux/select");
        nh_.param<std::string>("tracker_cmd_topic", tracker_cmd_topic_, "/quadrotor/tracker_cmd");
        mux_client_ = nh_.serviceClient<topic_tools::MuxSelect>(mux_service_name_);
        
        // Load parameters
        nh_.param("hover_height", hover_height_, 1.0);
        
        // Wait for mux service and switch to tracker_cmd initially
        ros::Duration(1.0).sleep();  // Wait for mux to start
        switchMux(false);  // Select tracker_cmd by default
        
        ROS_INFO("[TrajServer] Initialized, waiting for trajectory...");
    }

private:
    void trajectoryCallback(const trajectory_planner::BsplineTrajectory::ConstPtr& msg) {
        if (msg->knots.empty()) {
            ROS_WARN("[TrajServer] Received empty trajectory");
            return;
        }
        
        // Parse control points
        int num_pts = msg->knots.size() / 3;
        control_points_.clear();
        control_points_.resize(num_pts);
        
        for (int i = 0; i < num_pts; ++i) {
            control_points_[i] = Eigen::Vector3d(
                msg->knots[3*i], 
                msg->knots[3*i + 1], 
                msg->knots[3*i + 2]
            );
        }
        
        dt_ = msg->dt;
        duration_ = msg->duration;
        start_time_ = ros::Time::now();  // Start execution immediately
        
        has_traj_ = true;
        executing_ = true;
        
        // Switch mux to traj_server
        switchMux(true);
        
        ROS_INFO("[TrajServer] Received trajectory with %d control points, dt=%.3f, duration=%.2f",
                 num_pts, dt_, duration_);
    }
    
    void switchMux(bool to_traj_server) {
        topic_tools::MuxSelect srv;
        if (to_traj_server) {
            srv.request.topic = nh_.resolveName("position_cmd");
        } else {
            srv.request.topic = tracker_cmd_topic_;
        }
        
        if (mux_client_.call(srv)) {
            ROS_INFO("[TrajServer] Switched mux to: %s", srv.request.topic.c_str());
        } else {
            ROS_WARN("[TrajServer] Failed to switch mux (service may not be available)");
        }
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pos_ = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        current_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);
        has_odom_ = true;
    }
    
    void cmdTimerCallback(const ros::TimerEvent& event) {
        if (!has_odom_) return;
        
        // Don't publish until we receive a trajectory - let mav_services handle takeoff
        if (!has_traj_) return;
        
        kr_mav_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        
        if (executing_) {
            double t = (ros::Time::now() - start_time_).toSec();
            
            if (t >= duration_) {
                // Trajectory finished, hover at final position
                executing_ = false;
                Eigen::Vector3d final_pos = evaluatePosition(duration_ - 0.01);
                cmd.position.x = final_pos.x();
                cmd.position.y = final_pos.y();
                cmd.position.z = final_pos.z();
                cmd.velocity.x = 0;
                cmd.velocity.y = 0;
                cmd.velocity.z = 0;
                cmd.acceleration.x = 0;
                cmd.acceleration.y = 0;
                cmd.acceleration.z = 0;
                ROS_INFO_ONCE("[TrajServer] Trajectory finished, hovering at goal");
            } else {
                // Sample trajectory
                Eigen::Vector3d pos = evaluatePosition(t);
                Eigen::Vector3d vel = evaluateVelocity(t);
                Eigen::Vector3d acc = evaluateAcceleration(t);
                
                cmd.position.x = pos.x();
                cmd.position.y = pos.y();
                cmd.position.z = pos.z();
                cmd.velocity.x = vel.x();
                cmd.velocity.y = vel.y();
                cmd.velocity.z = vel.z();
                cmd.acceleration.x = acc.x();
                cmd.acceleration.y = acc.y();
                cmd.acceleration.z = acc.z();
            }
        } else {
            // Trajectory completed, hover at final position
            Eigen::Vector3d final_pos = evaluatePosition(duration_ - 0.01);
            cmd.position.x = final_pos.x();
            cmd.position.y = final_pos.y();
            cmd.position.z = final_pos.z();
            cmd.velocity.x = 0;
            cmd.velocity.y = 0;
            cmd.velocity.z = 0;
            cmd.acceleration.x = 0;
            cmd.acceleration.y = 0;
            cmd.acceleration.z = 0;
        }
        
        // Yaw: keep current
        cmd.yaw = current_yaw_;
        cmd.yaw_dot = 0;
        
        pos_cmd_pub_.publish(cmd);
    }
    
    // B-spline evaluation functions
    Eigen::Vector3d evaluatePosition(double t) const {
        return evaluateBspline(t, 0);
    }
    
    Eigen::Vector3d evaluateVelocity(double t) const {
        return evaluateBspline(t, 1);
    }
    
    Eigen::Vector3d evaluateAcceleration(double t) const {
        return evaluateBspline(t, 2);
    }
    
    Eigen::Vector3d evaluateBspline(double t, int derivative) const {
        if (control_points_.size() < 4) {
            return control_points_.empty() ? Eigen::Vector3d::Zero() : control_points_[0];
        }
        
        // Clamp time
        t = std::max(0.0, std::min(t, duration_ - 0.001));
        
        // Find segment index
        int n = control_points_.size() - 3;  // number of segments for cubic B-spline
        int seg_idx = std::min(static_cast<int>(t / dt_), n - 1);
        double u = (t - seg_idx * dt_) / dt_;
        
        // Cubic B-spline basis matrix
        Eigen::Matrix4d M;
        M << -1,  3, -3,  1,
              3, -6,  3,  0,
             -3,  0,  3,  0,
              1,  4,  1,  0;
        M /= 6.0;
        
        // Derivative factor
        double factor = 1.0;
        Eigen::Vector4d U;
        
        if (derivative == 0) {
            U << u*u*u, u*u, u, 1;
        } else if (derivative == 1) {
            U << 3*u*u, 2*u, 1, 0;
            factor = 1.0 / dt_;
        } else if (derivative == 2) {
            U << 6*u, 2, 0, 0;
            factor = 1.0 / (dt_ * dt_);
        } else {
            U << 6, 0, 0, 0;
            factor = 1.0 / (dt_ * dt_ * dt_);
        }
        
        // Get 4 control points for this segment
        Eigen::Matrix<double, 4, 3> P;
        for (int i = 0; i < 4; ++i) {
            P.row(i) = control_points_[seg_idx + i].transpose();
        }
        
        Eigen::Vector3d result = factor * (U.transpose() * M * P).transpose();
        return result;
    }
    
    double getYawFromQuaternion(const geometry_msgs::Quaternion& q) const {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
    
    ros::NodeHandle nh_;
    ros::Publisher pos_cmd_pub_;
    ros::Subscriber traj_sub_;
    ros::Subscriber odom_sub_;
    ros::Timer cmd_timer_;
    
    // Trajectory data
    std::vector<Eigen::Vector3d> control_points_;
    double dt_ = 0.1;
    double duration_ = 0.0;
    ros::Time start_time_;
    bool has_traj_ = false;
    bool executing_ = false;
    
    // Odometry
    Eigen::Vector3d current_pos_ = Eigen::Vector3d::Zero();
    double current_yaw_ = 0.0;
    bool has_odom_ = false;
    
    // Parameters
    double hover_height_ = 1.0;
    
    // Mux
    ros::ServiceClient mux_client_;
    std::string mux_service_name_;
    std::string tracker_cmd_topic_;
};

}  // namespace trajectory_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_server_node");
    ros::NodeHandle nh("~");
    
    trajectory_planner::TrajServer server(nh);
    
    ros::spin();
    return 0;
}
