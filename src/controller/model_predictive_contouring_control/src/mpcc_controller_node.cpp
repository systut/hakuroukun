#include <ros/ros.h>
#include <cmath>
#include <mutex>
#include <boost/make_shared.hpp>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <std_msgs/Float64MultiArray.h>

#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"

#include "model_predictive_contouring_control/spline_test.h"
#include "model_predictive_contouring_control/model_integrator_test.h"
#include "model_predictive_contouring_control/constraints_test.h"
#include "model_predictive_contouring_control/cost_test.h"

#include "model_predictive_contouring_control/mpc.h"
#include "model_predictive_contouring_control/integrator.h"
#include "model_predictive_contouring_control/track.h"
#include "model_predictive_contouring_control/plotting.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;

using namespace mpcc;

class MpccControllerNode
{
public:

    MpccControllerNode(ros::NodeHandle nh, ros::NodeHandle private_nh) {
        
        Init(nh, private_nh);
    }

    void RunMPCC() {

        std::ifstream iConfig("src/controller/model_predictive_contouring_control/config/config.json");
        json jsonConfig;
        iConfig >> jsonConfig;

        PathToJson json_paths {jsonConfig["model_path"],
                            jsonConfig["cost_path"],
                            jsonConfig["bounds_path"],
                            jsonConfig["track_path"],
                            jsonConfig["normalization_path"]};

        Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
        Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);
        ArcLengthSpline arc_length_spline = ArcLengthSpline(json_paths);

        std::list<MPCReturn> log;
        MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);

        // =====  SET RUNNING TRACK  ======================================================================================
        std::vector<double> x, y;
        for(int i = 0; i < trajectory_.points.size(); i++){
            x.push_back(trajectory_.points.at(i).x);
            y.push_back(trajectory_.points.at(i).y);
        }

        Eigen::Vector2d goal_point;
        goal_point << trajectory_.points.at(trajectory_.points.size()-1).x, trajectory_.points.at(trajectory_.points.size()-1).y;
        
        Track track = Track();
        // Track track = Track(json_paths.track_path);
        track.setTrack(x, y);
        TrackPos track_xy = track.getTrack();

        // ==== Set MPC Problems ================================================================================================

        mpc.setTrack(track_xy.X,track_xy.Y);

        double ts = jsonConfig["Ts"];
        int i = 0;
        ros::Rate r(1.0f/ts);
        double s = 0;
        double v0 = 0.5;
        double delta0 = 0.0;
        double vs = 0.5;
        // double dV, dDelta, dVs;
        // const double phi_0 = std::atan2(pose_y - track_xy.Y(0),pose_x - track_xy.X(0));
        
        State x0 = {front_x, front_y, yaw, s, v0, delta0, vs};    
        // State x0 = {pose_x, pose_y, yaw, s, v0, delta0, vs};    

        // State x0 = {track_xy.X(0),track_xy.Y(0),phi_0,0,jsonConfig["v0"],0,jsonConfig["v0"]};
        // std::cout << "X : " << track_xy.X(0) << " Y : " << track_xy.Y(0) << " Phi : " << phi_0  << " s : " << s << " V : " << jsonConfig["v0"] << " Delta : "<< delta0 << " Vs : " << jsonConfig["v0"] <<std::endl;
        arc_length_spline.gen2DSpline(track_xy.X,track_xy.Y);
        // ===========================
        // RUN MPCC
        // ===========================

        while (ros::ok())
        {
            std::cout << "==== " << "Step : " << i << " ======"  << std::endl;
            std::cout << "X : " << x0.X << " Y : " << x0.Y << " Phi : " << x0.phi  << " s : " << x0.s << " V : " << x0.v << " Delta : "<< x0.delta << " Vs : " << x0.vs <<std::endl;
            std::cout << "========================="  << std::endl;

            x0.X = front_x;
            x0.Y = front_y;
            // x0.X = pose_x;
            // x0.Y = pose_y;
            x0.phi = yaw;
            x0.s = arc_length_spline.projectOnSpline(x0);

            x0.v = v0;
            x0.delta = delta0;
            x0.vs = vs;

            Eigen::Vector2d current_robot_pose;
            current_robot_pose << front_x, front_y;
            // current_robot_pose << pose_x, pose_y;
            Eigen::Vector2d error = goal_point - current_robot_pose;

            if(atGoal(error))
            {
                break;
            }
            else
            {
                MPCReturn mpc_sol = mpc.runMPC(x0);

                v0 = v0 + mpc_sol.u0.dV * ts;
                delta0 = delta0 + mpc_sol.u0.dDelta * ts;
                vs = vs + mpc_sol.u0.dVs * ts;

                std::cout << "Delta : " << delta0 << std::endl;


                controller_msg.data[0] = v0;
                controller_msg.data[1] = delta0;
                
                controller_publisher.publish(controller_msg);

                // State x0_new = integrator.simTimeStep(x0, mpc_sol.u0, ts);
                
                // log.push_back(mpc_sol);

                // MPCReturn mpc_sol = mpc.runMPC(x0);
                // x0 = integrator.simTimeStep(x0, mpc_sol.u0, ts);

                // controller_msg.data[0] = x0.v;
                // controller_msg.data[1] = x0.delta;

                // std::cout << "V : " << x0.v << ", Delta : " << x0.delta << std::endl; 
                
                // controller_publisher.publish(controller_msg);

                // State x0_new = integrator.simTimeStep(x0, mpc_sol.u0, ts);
                
                log.push_back(mpc_sol);
            }

            i++;
            // if(jsonConfig["n_sim"] < i)
            // {
            //     break;
            // }

            ros::spinOnce();
            r.sleep();
        }


        controller_msg.data[0] = 0.0;
        controller_msg.data[1] = 0.0;
        
        controller_publisher.publish(controller_msg);

        // ===========================
        // PLOTTING
        // ===========================

        plotter.plotRun(log,track_xy);
        // plotter.plotSim(log,track_xy);
    
        // double mean_time = 0.0;
        // double max_time = 0.0;
        // for(MPCReturn log_i : log)
        // {
        //     mean_time += log_i.time_total;
        //     if(log_i.time_total > max_time)
        //         max_time = log_i.time_total;
        // }
        // std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
        // std::cout << "max nmpc time " << max_time << std::endl;
    }

    void Simulation()
    {
        std::ifstream iConfig("src/controller/model_predictive_contouring_control/config/config.json");
        json jsonConfig;
        iConfig >> jsonConfig;

        PathToJson json_paths {jsonConfig["model_path"],
                            jsonConfig["cost_path"],
                            jsonConfig["bounds_path"],
                            jsonConfig["track_path"],
                            jsonConfig["normalization_path"]};

        Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
        Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);
        ArcLengthSpline arc_length_spline = ArcLengthSpline(json_paths);

        std::list<MPCReturn> log;
        MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);

        // =====  SET RUNNING TRACK  ======================================================================================
        std::vector<double> x, y;
        for(int i = 0; i < trajectory_.points.size(); i++){
            x.push_back(trajectory_.points.at(i).x);
            y.push_back(trajectory_.points.at(i).y);
        }

        
        Track track = Track();
        // Track track = Track(json_paths.track_path);
        track.setTrack(x, y);
        TrackPos track_xy = track.getTrack();

        // ==== Set MPC Problems ================================================================================================

        mpc.setTrack(track_xy.X,track_xy.Y);

        const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));

        State x0 = {track_xy.X(0),track_xy.Y(0),phi_0,0,1,0,1};
        // std::cout << "X : " << track_xy.X(0) << " Y : " << track_xy.Y(0) << " Phi : " << phi_0  << " s : " << s << " V : " << jsonConfig["v0"] << " Delta : "<< delta0 << " Vs : " << jsonConfig["v0"] <<std::endl;

        // RUN MPCC
        for(int i=0;i<jsonConfig["n_sim"];i++)
        {
            if (i < 100)
            {
                std::cout << "==== " << "Step : " << i << " ======"  << std::endl;
                std::cout << "X : " << x0.X << " Y : " << x0.Y << " Phi : " << x0.phi  << " s : " << x0.s << " V : " << x0.v << " Delta : "<< x0.delta << " Vs : " << x0.vs <<std::endl;
                std::cout << "========================="  << std::endl;

            }

            // TODO From MPC Return -> Get Input for robot and set goal tolerance
            MPCReturn mpc_sol = mpc.runMPC(x0);

            // Update new State    
            x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
            
            // Logger
            log.push_back(mpc_sol);
        }
    
        // ===========================
        // PLOTTING
        // ===========================

        // plotter.plotRun(log,track_xy);
        // plotter.plotSim(log,track_xy);
    
        // double mean_time = 0.0;
        // double max_time = 0.0;
        // for(MPCReturn log_i : log)
        // {
        //     mean_time += log_i.time_total;
        //     if(log_i.time_total > max_time)
        //         max_time = log_i.time_total;
        // }
        // std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
        // std::cout << "max nmpc time " << max_time << std::endl;
    
    }


private:
    // ROS Node / Publishers / Subscribers
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber emergency_flag_sub_;
    ros::Subscriber orientation_sub_;
    ros::Publisher controller_publisher;
    std_msgs::Float64MultiArray controller_msg;
    // ROS Message
    geometry_msgs::PoseStamped odom_msg_;
    std_msgs::Bool emergency_flag_msg_;
    std_msgs::Float64 orientation_msg_;

    Eigen::Vector3d robot_pose_;
    Eigen::Vector3d front_pose_;
    sdv_msgs::Trajectory trajectory_;
    
    double l_f = 0.95;
    double l_b = 0.55;
    double front_x, front_y;
    double back_x, back_y;
    double pose_x, pose_y, yaw;
    double sampling_time_;



    // ===============================================
    // Callback function
    // ===============================================
    void OrientationCallback(const std_msgs::Float64 &orientation_msg)
    {
        orientation_msg_ = orientation_msg;
    
        double pi = 3.14159;
            
        yaw = floorf((orientation_msg_.data * (pi / 180) * 100.0)) / 100.0;
    }

    void OdomCallback(const geometry_msgs::PoseStamped &odom_msg) {
        
        odom_msg_ = odom_msg;
     
        OdomToPose(odom_msg_);
    }

    void OdomToPose(const geometry_msgs::PoseStamped &odom_msg)
    {
        pose_x = floorf(odom_msg.pose.position.x * 100.0) / 100.0;
     
        pose_y = floorf(odom_msg.pose.position.y * 100.0) / 100.0;

        front_x = floorf((pose_x + l_f * cos(yaw)) * 100.0) / 100.0;

        front_y = floorf((pose_y + l_f * sin(yaw)) * 100.0) / 100.0;

        back_x = pose_x - l_b * cos(yaw);

        back_y = pose_y - l_b * sin(yaw);
    }
    
    void EmergencyFlagCallback(const std_msgs::Bool &emergency_flag_msg)
    {
        emergency_flag_msg_ = emergency_flag_msg;
    }

    // ===============================================
    // ---- function
    // ===============================================
    bool atGoal(const Eigen::Vector2d &error)
    {   

        if (std::abs(error(0)) < 0.5 && std::abs(error(1)) < 0.5)
        {
            return true;
        }

        return false;
    }


    void Init(ros::NodeHandle nh, ros::NodeHandle private_nh){

        nh_ = nh;
        private_nh_ = private_nh;
        nh.param("controller_sampling_time", sampling_time_, 0.05);

        ROS_INFO("mpcc_controller_node initialized ... Start localizing");

        odom_sub_ = nh_.subscribe("/hakuroukun_pose/pose", 4, &MpccControllerNode::OdomCallback, this);
        orientation_sub_ = nh_.subscribe("/hakuroukun_pose/orientation", 4, &MpccControllerNode::OrientationCallback, this);
        // emergency_flag_sub_ = nh.subscribe("/emergency_flag", 1, &MpccControllerNode::EmergencyFlagCallback, this);
        controller_publisher = nh_.advertise<std_msgs::Float64MultiArray>("cmd_controller", 1000);
        controller_msg.data = {0.0, 0.0};

        boost::shared_ptr<sdv_msgs::Trajectory const> traj_msg;
        traj_msg = ros::topic::waitForMessage<sdv_msgs::Trajectory>("/trajectory", nh_);
        if (traj_msg != NULL)
        {
            trajectory_ = *traj_msg;
        }

        ROS_INFO("trajectory received.... Start initializing mpcc controller");
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpcc_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    MpccControllerNode mpcc_controller_node(nh, private_nh);

    mpcc_controller_node.RunMPCC();
    // mpcc_controller_node.Simulation();

    // ros::AsyncSpinner s(4);
    // s.start();
    // ros::waitForShutdown();

    return 0;
}