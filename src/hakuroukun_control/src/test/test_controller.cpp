#include <cmath>
#include <mutex>
#include <boost/make_shared.hpp>
#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include "../../include/mpc.h"
#include "../../include/model.h"

void saveData(Eigen::VectorXd vector)
{
        //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
        const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
        
        std::ofstream file("src/data/result/test_controller.csv");
        if (file.is_open())
        {
                file << vector.format(CSVFormat);
                file.close();
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_controller_node");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_("~");
    double sampling_time_ = 0.05;

    sdv_msgs::Trajectory trajectory_;
    boost::shared_ptr<sdv_msgs::Trajectory const> trajectory_msg;
    trajectory_msg = ros::topic::waitForMessage<sdv_msgs::Trajectory>("/trajectory", nh_);
    if (trajectory_msg != NULL)
    {
        trajectory_ = *trajectory_msg;
    }

    MPC mpc(nh_, private_nh_, sampling_time_);
    mpc.setTrajectory(trajectory_);
    Model model(sampling_time_);

    Eigen::Vector3d robot_pose ;
    robot_pose << 0.0, 0.0, 0.0;
    mpc.StraightMotion(robot_pose);

    // std::ofstream myfile;
    // myfile.open ("src/data/result/test_controller.csv");
    // for (size_t i = 0; i < mpc.iterations_; i++)
    // {
    //     mpc.GetHorizonTrajectory(i);
    //     // std::cout << mpc.x_N_ << std::endl;
    //     std::tie(mpc.Aeq_, mpc.Beq_) =mpc.SetEqualityConstraints(mpc.x_N_, mpc.u_N_, robot_pose);

    //     Eigen::VectorXd optimal_solution(model.nx);
    //     optimal_solution = mpc.SolveMPCProblem(mpc.H_, mpc.f_, mpc.Aeq_, mpc.Beq_, mpc.Ai_, mpc.Bi_);
        
    //     robot_pose = model.DynamicFunction(robot_pose, optimal_solution);
    //     // std::cout << "current robot pose :" <<robot_pose << std::endl;

    //     std::stringstream ss;
    //     ss <<  robot_pose(0) << "," << robot_pose(1) << "," << robot_pose(2) << ", \n"; 
    //     // std::cout << ss.str() << std::endl;
    //     myfile << ss.str();
    // }
    // myfile.close();    

    // return 0;
}
