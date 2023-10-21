#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <mutex>
#include <boost/make_shared.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"
#include <std_msgs/Float64MultiArray.h>

#include "../../include/mpc.h"
#include "../../include/model.h"
#include "../../include/QuadProg++.h"

void saveMatrix(Eigen::MatrixXd matrix, std::string name)
{
        //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
        const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
        
        std::ofstream file("src/data/result/" + name + ".csv");
        if (file.is_open())
        {
                file << matrix.format(CSVFormat);
                file.close();
        }
};

void saveVector(Eigen::VectorXd vector, std::string name)
{
        //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
        const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
        
        std::ofstream file("src/data/result/" + name + ".csv");
        if (file.is_open())
        {
                file << vector.format(CSVFormat);
                file.close();
        }
};


int main(int argc, char **argv)
{
        ros::init(argc, argv, "test_mpc_node");
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        // -----------------------------------------------------
        double sampling_time_ = 0.05;

        sdv_msgs::Trajectory trajectory_msg_;

        trajectory_msg_.points.resize(10);
        trajectory_msg_.points[0].x = 0.0;
        trajectory_msg_.points[0].y = 0.0;
        trajectory_msg_.points[0].heading = 0.0;
        trajectory_msg_.points[0].velocity_mps = 1;
        trajectory_msg_.points[0].steering_angle = 0.0;

        trajectory_msg_.points[1].x = 0.0506;
        trajectory_msg_.points[1].y = 0.0;
        trajectory_msg_.points[1].heading = 0.0;
        trajectory_msg_.points[1].velocity_mps = 1;
        trajectory_msg_.points[1].steering_angle = 0.0;

        trajectory_msg_.points[2].x = 0.1011;
        trajectory_msg_.points[2].y = 0.0;
        trajectory_msg_.points[2].heading = 0.0;
        trajectory_msg_.points[2].velocity_mps = 1;
        trajectory_msg_.points[2].steering_angle = 0.0;

        trajectory_msg_.points[3].x = 0.1517;
        trajectory_msg_.points[3].y = 0.0;
        trajectory_msg_.points[3].heading = 0.0;
        trajectory_msg_.points[3].velocity_mps = 1;
        trajectory_msg_.points[3].steering_angle = 0.0;

        trajectory_msg_.points[4].x = 0.2022;
        trajectory_msg_.points[4].y = 0.0;
        trajectory_msg_.points[4].heading = 0.0;
        trajectory_msg_.points[4].velocity_mps = 1;
        trajectory_msg_.points[4].steering_angle = 0.0;
        
        trajectory_msg_.points[5].x = 0.2528;
        trajectory_msg_.points[5].y = 0.0;
        trajectory_msg_.points[5].heading = 0.0;
        trajectory_msg_.points[5].velocity_mps = 1;
        trajectory_msg_.points[5].steering_angle = 0.0;

        trajectory_msg_.points[6].x = 0.3034;
        trajectory_msg_.points[6].y = 0.0;
        trajectory_msg_.points[6].heading = 0.0;
        trajectory_msg_.points[6].velocity_mps = 1;
        trajectory_msg_.points[6].steering_angle = 0.0;

        trajectory_msg_.points[7].x = 0.3539;
        trajectory_msg_.points[7].y = 0.0;
        trajectory_msg_.points[7].heading = 0.0;
        trajectory_msg_.points[7].velocity_mps = 1;
        trajectory_msg_.points[7].steering_angle = 0.0;

        trajectory_msg_.points[8].x = 0.4045;
        trajectory_msg_.points[8].y = 0.0;
        trajectory_msg_.points[8].heading = 0.0;
        trajectory_msg_.points[8].velocity_mps = 1;
        trajectory_msg_.points[8].steering_angle = 0.0;

        trajectory_msg_.points[9].x = 0.4551;
        trajectory_msg_.points[9].y = 0.0;
        trajectory_msg_.points[9].heading = 0.0;
        trajectory_msg_.points[9].velocity_mps = 1;
        trajectory_msg_.points[9].steering_angle = 0.0;

        // trajectory_msg_.points[10].x = 0.5056;
        // trajectory_msg_.points[10].y = 0.0;
        // trajectory_msg_.points[10].heading = 0.0;
        // trajectory_msg_.points[10].velocity_mps = 1;
        // trajectory_msg_.points[10].steering_angle = 0.0;

        // -----------------------------------------------------
        MPC mpc(nh, private_nh, sampling_time_, trajectory_msg_);
        Model model(sampling_time_);

        Eigen::MatrixXd x_ref(model.nx, mpc.predict_steps_+1);
        Eigen::MatrixXd u_ref(model.nu, mpc.predict_steps_+1);
        Eigen::Vector3d y_out;

        x_ref<< 0.0, 0.0506, 0.1011, 0.1517, 0.2022, 0.2528, 0.3034, 0.3539, 0.4045, 0.4551, 0.5056,
                0.0, 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ,
                0.0, 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ;

        u_ref<< 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        y_out<< 0.0,
                0.0,
                0.0;

        Eigen::MatrixXd A_in(2*model.nu*(mpc.predict_steps_-1), ((mpc.predict_steps_+1)*model.nx + (mpc.predict_steps_)*model.nu));
        Eigen::VectorXd B_in(2*model.nu*(mpc.predict_steps_-1));
        std::tie(A_in, B_in) = mpc.SetInequalityConstraints();

        Eigen::MatrixXd A_eq(model.nx*(mpc.predict_steps_+1), model.nx*(mpc.predict_steps_+1)+model.nu*mpc.predict_steps_);
        Eigen::VectorXd B_eq(model.nx*(mpc.predict_steps_+1));
        std::tie(A_eq, B_eq) = mpc.SetEqualityConstraints(x_ref, u_ref, y_out);

        Eigen::MatrixXd H(model.nx*(mpc.predict_steps_+1) + model.nu*mpc.predict_steps_, model.nx*(mpc.predict_steps_+1) + model.nu*mpc.predict_steps_);
        Eigen::VectorXd f(model.nx*(mpc.predict_steps_+1) + model.nu*mpc.predict_steps_);
        std::tie(H, f) = mpc.SetStageCost();

        saveMatrix(H, "H");
        saveVector(f, "f");

        saveMatrix(A_eq, "A_eq");
        saveVector(B_eq, "B_eq");

        saveMatrix(A_in, "A_in");
        saveVector(B_in, "B_in");

        Eigen::Vector2d optimal_solution = mpc.SolveMPCProblem(H,f,A_eq,B_eq,A_in,B_in);

        saveVector(optimal_solution, "u_opt");

        return 0;

}