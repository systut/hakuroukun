/*
 * ===============================================================================
 * mp_controller.cpp
 * Author: Dinh Ngoc Duc
 * Date: 06.10.23
 * Email: duc.dn.st@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This is the main node for the contrller of Hakuroukun. 
 * It receives information from the task manager and also information of the robot pose 
 * and the current encoder values. It calls the controller in a fixed sampling time.
 * ===============================================================================
 */

#include <ros/ros.h>
#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_pub_node");
    ros::NodeHandle nh;
    // ros::NodeHandle private_nh("~");

    ros::Publisher traj_pub_;
    ros::Rate loop_rate(10);

    // messages
    sdv_msgs::Trajectory trajectory_msg_;

    traj_pub_ = nh.advertise<sdv_msgs::Trajectory>("/trajectory", 1);

    trajectory_msg_.points.resize(4);
    trajectory_msg_.points[0].x = 1;
    trajectory_msg_.points[0].y = 1;
    trajectory_msg_.points[0].heading = 1;
    trajectory_msg_.points[0].velocity_mps = 1;
    trajectory_msg_.points[0].steering_angle = 1;

    trajectory_msg_.points[1].x = 2;
    trajectory_msg_.points[1].y = 2;
    trajectory_msg_.points[1].heading = 2;
    trajectory_msg_.points[1].velocity_mps = 2;
    trajectory_msg_.points[1].steering_angle = 2;


    trajectory_msg_.points[2].x = 3;
    trajectory_msg_.points[2].y = 3;
    trajectory_msg_.points[2].heading = 3;
    trajectory_msg_.points[2].velocity_mps = 3;
    trajectory_msg_.points[2].steering_angle = 3;


    trajectory_msg_.points[3].x = 4;
    trajectory_msg_.points[3].y = 4;
    trajectory_msg_.points[3].heading = 4;
    trajectory_msg_.points[3].velocity_mps = 4;
    trajectory_msg_.points[3].steering_angle = 4;


    while (ros::ok())
    {
        traj_pub_.publish(trajectory_msg_);
    }

}