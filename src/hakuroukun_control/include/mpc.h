/*The problem is in the form:

min 0.5 * x G x + g0 x
s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0
	 
 The matrix and vectors dimensions are as follows:
     G: n * n
		g0: n
				
		CE: n * p
	 ce0: p
				
	  CI: n * m
   ci0: m

     x: n
 
 The function will return the cost of the solution written in the x vector or
 std::numeric_limits::infinity() if the problem is infeasible. In the latter case
 the value of the x vector is not correct.
 
 References: D. Goldfarb, A. Idnani. A numerically stable dual method for solving
             strictly convex quadratic programs. Mathematical Programming 27 (1983) pp. 1-33.

 Notes:
  1. pay attention in setting up the vectors ce0 and ci0. 
	   If the constraints of your problem are specified in the form 
	   A^T x = b and C^T x >= d, then you should set ce0 = -b and ci0 = -d.
  2. The matrices have column dimension equal to MATRIX_DIM, 
     a constant set to 20 in this file (by means of a #define macro). 
     If the matrices are bigger than 20 x 20 the limit could be
		 increased by means of a -DMATRIX_DIM=n on the compiler command line.
  3. The matrix G is modified within the function since it is used to compute
     the G = L^T L cholesky factorization for further computations inside the function. 
     If you need the original matrix G you should make a copy of it and pass the copy
     to the function.
*/

#ifdef MPC_H
#define MPC_H
#endif // MPC_H

#include <cmath>
#include <algorithm>
#include <mutex>
#include <vector>
#include <string>
#include <fstream>
#include <time.h>
#include <ctime>
#include <tuple>

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"

// #include "model.h"
#include "QuadProg++.h"


class MPC
{
    
public:

    MPC(){}
    
    MPC(ros::NodeHandle nh, ros::NodeHandle private_nh, double sampling_time);

    void setTrajectory(sdv_msgs::Trajectory &traj);
    void GetHorizonTrajectory(double current_step);
    void Control(Eigen::Vector3d robot_pose);
    void StopMotion(Eigen::Vector3d robot_pose);
    void StraightMotion(Eigen::Vector3d robot_pose);
    void Control2(Eigen::Vector3d robot_pose);

    void StraightMotionWithKp(Eigen::Vector3d robot_pose);
    void StraightMotionWithStanley(Eigen::Vector3d robot_pose);
    
    // ==============================================
    // P - Controller for Straight motion
    // ==============================================
    double Kp;
    void Control3(Eigen::Vector3d robot_pose);
    void Control4(Eigen::Vector3d gps_pose);

    // ==============================================
    // MPC FUNCTIONS
    // ==============================================
    std::tuple<Eigen::MatrixXd, Eigen::VectorXd> SetStageCost();
    std::tuple<Eigen::MatrixXd, Eigen::VectorXd> SetInequalityConstraints();
    std::tuple<Eigen::MatrixXd, Eigen::VectorXd> SetEqualityConstraints(Eigen::MatrixXd x_ref, 
                                                                        Eigen::MatrixXd u_ref, 
                                                                        Eigen::Vector3d y_out);

    Eigen::VectorXd SolveMPCProblem(Eigen::MatrixXd H, Eigen::VectorXd f,
                                    Eigen::MatrixXd A_eq, Eigen::VectorXd B_eq,
                                    Eigen::MatrixXd A_in, Eigen::VectorXd B_in);


    int nx_;
    int nu_;
    int iterations_;
    int predict_steps_;
    double sampling_time_;
    int counter_;
    std::vector<bool> curve_detect;

    // Horizontal Reference Trajectory
    Eigen::MatrixXd x_N_;
    Eigen::MatrixXd u_N_;
    Eigen::VectorXd x_N_goal_;

    // Set Cost Matrix
    Eigen::MatrixXd H_;
    Eigen::MatrixXd f_;

    // Set Inequality constraints
    Eigen::MatrixXd Ai_;
    Eigen::MatrixXd Bi_;

    // Set Equality constraints
    Eigen::MatrixXd Aeq_;
    Eigen::MatrixXd Beq_;

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher controller_publisher;
    std_msgs::Float64MultiArray controller_msg;

    void ReadTrajectory();
    void SetMPCParameters();
    void PublishControlCommand(Eigen::Vector2d input);
    void GenerateCSV(const Eigen::Vector3d pose, const Eigen::Vector2d input);
    
    // ==============================================
    // MPC FUNCTIONS
    // ==============================================
    // std::tuple<Eigen::MatrixXd, Eigen::VectorXd> SetStageCost();
    // std::tuple<Eigen::MatrixXd, Eigen::VectorXd> SetInequalityConstraints();
    // std::tuple<Eigen::MatrixXd, Eigen::VectorXd> SetEqualityConstraints(Eigen::MatrixXd x_ref, 
    //                                                                     Eigen::MatrixXd u_ref, 
    //                                                                     Eigen::Vector3d y_out);
    // Eigen::VectorXd SolveMPCProblem(Eigen::MatrixXd H, Eigen::VectorXd f,
    //                             Eigen::MatrixXd A_eq, Eigen::Vector B_eq,
    //                             Eigen::MatrixXd A_in, Eigen::Vector A_in);

    // ==============================================
    // VARIABLES
    // ==============================================
    
    // double sampling_time_;
    sdv_msgs::Trajectory trajectory_;
    
    // Reference Trajectory
    Eigen::MatrixXd x_ref_;
    Eigen::MatrixXd u_ref_;
    Eigen::VectorXd end_goal_;

    // // Horizontal Reference Trajectory
    // Eigen::MatrixXd x_N_;
    // Eigen::MatrixXd u_N_;

    // // Set Cost Matrix
    // Eigen::MatrixXd H_;
    // Eigen::MatrixXd f_;

    // // Set Inequality constraints
    // Eigen::MatrixXd Ai_;
    // Eigen::MatrixXd Bi_;

    // MPC Solution
    Eigen::VectorXd optimal_solution_;

    char filename_[30];

    // Curve detection
    bool curve_detection;
};
