#include <ros/ros.h>
#include <eigen3/Eigen/Core>

#include "../include/mpc.h"
#include "../include/model.h"
#include <cmath>
#include <mutex>
#include <boost/make_shared.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>


class ControllerNode
{
public:

    ControllerNode(ros::NodeHandle nh, ros::NodeHandle private_nh) {
        Init(nh, private_nh);
        InitController();
    }

    void CallController(const ros::TimerEvent &event){

        // ===============================================
        // Need to add Stop motion and emergency stop here
        // ===============================================

        robot_pose_ << pose_x, pose_y, yaw;

        front_pose_ << front_x, front_y, yaw;

        // model_predictive_controller_.Control(robot_pose_);
        // model_predictive_controller_.Control2(robot_pose_);
        // model_predictive_controller_.Control3(robot_pose_);
        model_predictive_controller_.Control4(front_pose_);
        // model_predictive_controller_.StraightMotion(robot_pose_);
        // model_predictive_controller_.StraightMotionWithKp(robot_pose_);
        // model_predictive_controller_.StraightMotionWithStanley(robot_pose_);
    }


private:
    // ROS Node / Publishers / Subscribers
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber emergency_flag_sub_;
    ros::Subscriber orientation_sub_;
    ros::Timer periodic_timer_;
    // ROS Message
    geometry_msgs::PoseStamped odom_msg_;
    std_msgs::Bool emergency_flag_msg_;
    std_msgs::Float64 orientation_msg_;

    Eigen::Vector3d robot_pose_;
    Eigen::Vector3d front_pose_;
    sdv_msgs::Trajectory trajectory_;
    
    double l = 0.4;
    double front_x, front_y;
    double pose_x, pose_y, yaw;

    MPC model_predictive_controller_;
    double sampling_time_;

    void Init(ros::NodeHandle nh, ros::NodeHandle private_nh){

        nh_ = nh;
        private_nh_ = private_nh;
        nh.param("controller_sampling_time", sampling_time_, 0.05);

        boost::shared_ptr<sdv_msgs::Trajectory const> traj_msg;
        traj_msg = ros::topic::waitForMessage<sdv_msgs::Trajectory>("/trajectory", nh_);
        if (traj_msg != NULL)
        {
            trajectory_ = *traj_msg;
        }

        odom_sub_ = nh_.subscribe("/hakuroukun_pose/pose", 4, &ControllerNode::OdomCallback, this);
        orientation_sub_ = nh_.subscribe("/hakuroukun_pose/orientation", 4, &ControllerNode::OrientationCallback, this);
        // emergency_flag_sub_ = nh.subscribe("/emergency_flag", 1, &ControllerNode::EmergencyFlagCallback, this);
        periodic_timer_ = private_nh_.createTimer(ros::Duration(0.05), &ControllerNode::CallController, this);
    }

    void InitController() {
        model_predictive_controller_ = MPC(nh_, private_nh_, sampling_time_);
        model_predictive_controller_.setTrajectory(trajectory_);
    }

    // ===============================================
    // Callback function
    // ===============================================
    void OrientationCallback(const std_msgs::Float64 &orientation_msg)
    {
        orientation_msg_ = orientation_msg;
    
        double pi = 3.14159265359;
            
        yaw = orientation_msg_.data * (pi / 180);
    }

    void OdomCallback(const geometry_msgs::PoseStamped &odom_msg) {
        
        odom_msg_ = odom_msg;
     
        OdomToPose(odom_msg_);
    }

    void OdomToPose(const geometry_msgs::PoseStamped &odom_msg)
    {
        pose_x = std::round(odom_msg.pose.position.x * 100.0) / 100.0;
     
        pose_y = std::round(odom_msg.pose.position.y * 100.0) / 100.0;

        front_x = pose_x + l * cos(yaw);

        front_y = pose_y + l * sin(yaw);
    }
    
    void EmergencyFlagCallback(const std_msgs::Bool &emergency_flag_msg)
    {
        emergency_flag_msg_ = emergency_flag_msg;
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    ControllerNode controller_node(nh, private_nh);

    ros::AsyncSpinner s(4);
    s.start();
    ros::waitForShutdown();

    return 0;
}