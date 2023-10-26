#include <ros/ros.h>
#include <eigen3/Eigen/Core>

#include "../include/mpc.h"
#include "../include/model.h"
#include <cmath>
#include <mutex>
#include <boost/make_shared.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>


class ControllerNode
{
public:

    ControllerNode(ros::NodeHandle nh, ros::NodeHandle private_nh) {
        _init(nh, private_nh);
        _init_controller();
    }

    void _call_controller(const ros::TimerEvent &event){
        
        geometry_msgs::Twist control_input_;

        double yaw = tf::getYaw(odom_msg_.pose.orientation);
        // if (yaw < 0)
        // {yaw += 2.0 * M_PI;}
        if (yaw != yaw)
        {yaw = 0.0;}

        double pose_x = std::round(odom_msg_.pose.position.x * 100.0) / 100.0;
        double pose_y = std::round(odom_msg_.pose.position.y * 100.0) / 100.0;

        robot_pose_ <<  pose_x, 
                        pose_y, 
                        yaw;

        // std::cout << "X :" << robot_pose_(0) << std::endl;
        // std::cout << "Y :" <<robot_pose_(1) << std::endl;

        model_predictive_controller_.Control(robot_pose_);
 
    }
    
    void _odom_callback(const geometry_msgs::PoseStamped &odom_msg){
        
        odom_msg_ = odom_msg;
    }


private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber odom_sub_;
    ros::Timer periodic_timer_;
    MPC model_predictive_controller_;
    
    geometry_msgs::PoseStamped odom_msg_;

    // Trajectory
    double sampling_time_;
    sdv_msgs::Trajectory trajectory_;

    // Robot Pose
    Eigen::Vector3d robot_pose_;

    void _init(ros::NodeHandle nh, ros::NodeHandle private_nh){

        nh_ = nh;
        private_nh_ = private_nh;

        nh.param("controller_sampling_time", sampling_time_, 0.05);

        boost::shared_ptr<sdv_msgs::Trajectory const> traj_msg;
        traj_msg = ros::topic::waitForMessage<sdv_msgs::Trajectory>("/trajectory", nh_);
        if (traj_msg != NULL)
        {
            trajectory_ = *traj_msg;
        }

        odom_sub_ = nh_.subscribe("/hakuroukun_pose/pose", 4, &ControllerNode::_odom_callback, this);

        periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &ControllerNode::_call_controller, this);
    }

    void _init_controller() {

        model_predictive_controller_ = MPC(nh_, private_nh_, sampling_time_, trajectory_);
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