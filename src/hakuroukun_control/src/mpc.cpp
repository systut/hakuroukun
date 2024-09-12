/*
 * ===============================================================================
 * mpc.cpp
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

#include "../include/mpc.h"
#include "../include/model.h"

// ========================================================
// PUBLIC FUNCTION
// ========================================================
MPC::MPC(ros::NodeHandle nh, ros::NodeHandle private_nh, double sampling_time)
{
    // Set dt
    sampling_time_ = sampling_time;

    // MPC Parameters
    SetMPCParameters();

    // MPC Matrices
    std::tie(H_, f_) = SetStageCost();
    std::tie(Ai_, Bi_) = SetInequalityConstraints();

    counter_= 1;
    optimal_solution_ = Eigen::VectorXd(nu_); 

    controller_publisher = nh_.advertise<std_msgs::Float64MultiArray>("cmd_controller_input", 1000);
    controller_msg.data = {0.0, 0.0};

    time_t now = time(0);
    strftime(filename_, sizeof(filename_), "src/data/testrun/%Y%m%d_%H%M.csv", localtime(&now));
    
}

void MPC::setTrajectory(sdv_msgs::Trajectory &traj)
{
    // Predict horizon trajectory
    trajectory_.header = traj.header;
    trajectory_.points = std::vector<sdv_msgs::TrajectoryPoint>(traj.points.begin(), traj.points.end());
    ReadTrajectory();
}

void MPC::ReadTrajectory()
{
    x_ref_ = Eigen::MatrixXd(nx_, trajectory_.points.size());
    u_ref_ = Eigen::MatrixXd(nu_, trajectory_.points.size());
    end_goal_ = Eigen::VectorXd(nx_);

    for(int i=0; i<trajectory_.points.size(); i++)
    {
        // X_ref = [x,y,theta]
        x_ref_(0,i) = trajectory_.points.at(i).x;
        x_ref_(1,i) = trajectory_.points.at(i).y;
        x_ref_(2,i) = trajectory_.points.at(i).heading;
        
        // U_ref = [v, delta]
        u_ref_(0,i) = trajectory_.points.at(i).velocity_mps;
        u_ref_(1,i) = trajectory_.points.at(i).steering_angle;
        
        // Get curve detection base on steering angle
        if (u_ref_(1,i) != 0)
        {
            curve_detect.push_back(true);
        }
        else
        {
            curve_detect.push_back(false);
        }
            // std::cout << curve_detect[i] << std::endl;

    }

    end_goal_ = x_ref_.block(0, x_ref_.cols()-1, 3, 1);

    iterations_ = trajectory_.points.size() - predict_steps_;

}

void MPC::GetHorizonTrajectory(double current_step)
{
    x_N_ = Eigen::MatrixXd(nx_, predict_steps_+1);
    u_N_ = Eigen::MatrixXd(nu_, predict_steps_+1);

    for (size_t i = 0; i <= predict_steps_; i++)
    {
        x_N_(0,i) = x_ref_(0, current_step + i);    // x
        x_N_(1,i) = x_ref_(1, current_step + i);    // y
        x_N_(2,i) = x_ref_(2, current_step + i);    // theta
        
        u_N_(0,i) = u_ref_(0, current_step + i);    // v
        u_N_(1,i) = u_ref_(1, current_step + i);    // delta   
    }
    x_N_goal_ = x_N_.col(0);
}

void MPC::StopMotion(Eigen::Vector3d robot_pose)
{
    double linear_vel = 0;
    double steer_angle = 0;

    controller_msg.data[0] = linear_vel;
    controller_msg.data[1] = steer_angle;
    controller_publisher.publish(controller_msg);
}

// ========================================================
// PRIVATE FUNCTION
// ========================================================
void MPC::SetMPCParameters()
{
    Model model(sampling_time_);
    nx_ = model.nx;
    nu_ = model.nu;
    predict_steps_ = 20;
}

void MPC::PublishControlCommand(Eigen::Vector2d input)
{
    double linear_vel = input(0);
    double steer_angle = input(1);

    if (std::abs(steer_angle) < 0.01)
    {
        double angular_vel_ref = 0.0;
    }
    
    controller_msg.data[0] = linear_vel;
    controller_msg.data[1] = steer_angle;
    // std::cout << controller_msg.data[0] << "," << controller_msg.data[1] << std::endl;
    controller_publisher.publish(controller_msg);
}

void MPC::GenerateCSV(const Eigen::Vector3d robot_pose, const Eigen::Vector2d input)
{
    std::ofstream export_data;

   // Eigen::Vector3d error = pose - ref_.block(1, counter_, 3, 1);
   // Eigen::Vector3d state_ref = ref_.block(1, counter_, 3, 1);

    export_data.open(filename_, std::ios::out|std::ios::app);

    for (long ii = 0; ii < robot_pose.rows(); ++ii)
    {
        export_data << robot_pose(ii) << ", ";
    }
    for (long ii = 0; ii < input.rows(); ++ii)
    {
        export_data << input(ii);
        if (ii != (input.rows()-1))
        {
            export_data << ", ";
        }
    }

    export_data << std::endl;

}

// ========================================================
// MPC FUNCTIONS
// ========================================================
std::tuple<Eigen::MatrixXd, Eigen::VectorXd> MPC::SetStageCost(){

    Eigen::MatrixXd Q(nx_,nx_);
    Eigen::MatrixXd R(nu_,nu_);
    Q   <<  100.0,  0.0,    0.0,
            0.0,  100.0,    0.0,
            0.0,    0.0,    0.0001;

    R   <<  0.0001, 0,
            0, 0.0001;

    Eigen::MatrixXd Q_stack(nx_*(predict_steps_+1), nx_*(predict_steps_+1) + nu_*(predict_steps_));
    Eigen::MatrixXd R_stack(nu_*(predict_steps_), nx_*(predict_steps_+1) + nu_*(predict_steps_));

    for (size_t i = 0; i < predict_steps_; i++)
    {
        Eigen::MatrixXd zero_matrix_q1 = Eigen::MatrixXd::Zero(Q.rows(),nx_*i);
        Eigen::MatrixXd zero_matrix_q2 = Eigen::MatrixXd::Zero(Q.rows(), ((predict_steps_+1)*nx_ - Q.cols() - zero_matrix_q1.cols()));
        Eigen::MatrixXd zero_matrix_r  = Eigen::MatrixXd::Zero(Q.rows(), nu_*(predict_steps_));
        Eigen::MatrixXd Q_stack_temp(nx_, nx_*(predict_steps_+1) + nu_*(predict_steps_));
        Q_stack_temp << zero_matrix_q1, Q, zero_matrix_q2, zero_matrix_r;

        Q_stack.row(nx_*i) = Q_stack_temp.row(0);
        Q_stack.row(nx_*i+1) = Q_stack_temp.row(1);
        Q_stack.row(nx_*i+2) = Q_stack_temp.row(2);

        Eigen::MatrixXd zero_matrix_q  = Eigen::MatrixXd::Zero(R.rows(), nx_*(predict_steps_+1));
        Eigen::MatrixXd zero_matrix_r1 = Eigen::MatrixXd::Zero(R.rows(),nu_*i);
        Eigen::MatrixXd zero_matrix_r2 = Eigen::MatrixXd::Zero(R.rows(), ((predict_steps_)*nu_ - R.cols() - zero_matrix_r1.cols()));
        Eigen::MatrixXd R_stack_temp(nu_, nx_*(predict_steps_+1) + nu_*(predict_steps_));
        R_stack_temp << zero_matrix_q, zero_matrix_r1, R, zero_matrix_r2;

        R_stack.row(nu_*i) = R_stack_temp.row(0);
        R_stack.row(nu_*i+1) = R_stack_temp.row(1);
    }
    Q_stack.block(nx_*predict_steps_, 0, 3, nx_*(predict_steps_+1) + nu_*(predict_steps_)).setZero();

    Eigen::MatrixXd H(nx_*(predict_steps_+1) + nu_*predict_steps_, nx_*(predict_steps_+1) + nu_*predict_steps_);

    H << Q_stack, R_stack;

    Eigen::VectorXd f = Eigen::VectorXd::Zero(nx_*(predict_steps_+1) + nu_*predict_steps_);

    return std::make_tuple(H, f);
}


std::tuple<Eigen::MatrixXd, Eigen::VectorXd> MPC::SetInequalityConstraints()
{
    Eigen::MatrixXd S_(8, 2*nu_);
    S_<< 1.0, 0.0 , 0.0, 0.0,
        -1.0, 0.0 , 0.0, 0.0,
         0.0, 1.0 , 0.0, 0.0,
         0.0,-1.0 , 0.0, 0.0, 
         1.0, 0.0 ,-1.0, 0.0,
        -1.0, 0.0 , 1.0, 0.0,
         0.0, 1.0 , 0.0,-1.0,
         0.0,-1.0 , 0.0, 1.0;
         
    Eigen::MatrixXd K_(S_.rows(),1);

    double lin_vel_ub = 1.5;
    double lin_vel_lb = 0.0;
    double steer_ang_ub = 0.785398;
    double steer_ang_lb = 0.785398;
    double delta_lin_vel_limit = 1.5/sampling_time_;
    double delta_steer_ang_limit = steer_ang_lb / sampling_time_;
    
    K_ << lin_vel_ub, 
          lin_vel_lb, 
          steer_ang_ub, 
          steer_ang_lb,
          delta_lin_vel_limit,
          delta_lin_vel_limit,
          delta_steer_ang_limit,
          delta_steer_ang_limit ;

    Eigen::MatrixXd A_u(S_.rows()*(predict_steps_-1), ((predict_steps_)*nu_));
    Eigen::MatrixXd B_u(K_.rows()*(predict_steps_-1), 1);

    for (size_t i = 0; i < predict_steps_-1; i++)
    {
        Eigen::MatrixXd A_u_k(S_.rows(), ((predict_steps_)*nu_));
        Eigen::MatrixXd zero_matrix_1 = Eigen::MatrixXd::Zero(S_.rows(),nu_*i);
        Eigen::MatrixXd zero_matrix_2 = Eigen::MatrixXd::Zero(S_.rows(), ((predict_steps_)*nu_ - S_.cols() - zero_matrix_1.cols()));
        A_u_k << zero_matrix_1, S_, zero_matrix_2;

        for (size_t ii = 0; ii < S_.rows(); ii++)
        {
            A_u.row(S_.rows()*i+ii)   = A_u_k.row(ii);

            B_u.row(K_.rows()*i+ii)   = K_.row(ii);
        }  
    }

    Eigen::MatrixXd A_in(S_.rows()*(predict_steps_-1), ((predict_steps_+1)*nx_ + (predict_steps_)*nu_));
    Eigen::VectorXd B_in(K_.rows()*(predict_steps_-1));

    Eigen::MatrixXd A_x = Eigen::MatrixXd::Zero(S_.rows()*(predict_steps_-1),((predict_steps_+1)*nx_));
    A_in << A_x, A_u;
    B_in << B_u;

    return std::make_tuple(A_in, B_in);

}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd> MPC::SetEqualityConstraints(Eigen::MatrixXd x_ref, Eigen::MatrixXd u_ref, Eigen::Vector3d y_out)
{

    Eigen::MatrixXd A_eq_x(nx_*(predict_steps_), nx_*(predict_steps_+1)+nu_*predict_steps_);

    for (size_t i = 0; i < predict_steps_; i++)
    {
        Model model(sampling_time_);
        Eigen::MatrixXd A_d(nx_, nx_);
        Eigen::MatrixXd B_d(nx_, nu_);
        Eigen::Vector3d x_lin;
        Eigen::Vector2d u_lin;
        
        x_lin = x_ref.block(0, i+1, x_ref.rows(),1);
        u_lin = u_ref.block(0, i+1, u_ref.rows(),1);

        A_d = model.SystemMatrix(x_lin, u_lin);
        B_d = model.ControlMatrix(x_lin, u_lin);

        Eigen::MatrixXd zero_matrix_a1 = Eigen::MatrixXd::Zero(nx_,nx_*i);
        Eigen::MatrixXd I_d = Eigen::MatrixXd::Identity(nx_,nx_);
        Eigen::MatrixXd zero_matrix_a2 = Eigen::MatrixXd::Zero(nx_, ((predict_steps_+1)*nx_ - zero_matrix_a1.cols() - A_d.cols() - I_d.cols()));
        Eigen::MatrixXd A_u(nx_, nx_*(predict_steps_+1));
        A_u << zero_matrix_a1, A_d, -I_d, zero_matrix_a2;

        Eigen::MatrixXd zero_matrix_b1 = Eigen::MatrixXd::Zero(nx_,nu_*i);
        Eigen::MatrixXd zero_matrix_b2 = Eigen::MatrixXd::Zero(nx_, ((predict_steps_)*nu_ - zero_matrix_b1.cols() - B_d.cols()));
        Eigen::MatrixXd B_u(nx_, nu_*(predict_steps_));
        B_u << zero_matrix_b1, B_d, zero_matrix_b2;

        Eigen::MatrixXd A_eq_i(nx_, nx_*(predict_steps_+1)+nu_*predict_steps_);
        A_eq_i << A_u, B_u;
        A_eq_x.row(nx_*i) = A_eq_i.row(0);
        A_eq_x.row(nx_*i+1) = A_eq_i.row(1);
        A_eq_x.row(nx_*i+2) = A_eq_i.row(2);
    }
    
    Eigen::MatrixXd A_eq(nx_*(predict_steps_+1), nx_*(predict_steps_+1)+nu_*predict_steps_);
    Eigen::VectorXd B_eq(nx_*(predict_steps_+1));
 
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nx_, nx_);
    Eigen::MatrixXd zero_matrix_aeq = Eigen::MatrixXd::Zero(nx_,nx_*(predict_steps_+1)+nu_*predict_steps_ - I.cols());
    A_eq << I, zero_matrix_aeq, A_eq_x;

    Eigen::VectorXd zero_matrix_bep = Eigen::VectorXd::Zero(A_eq.rows() - y_out.rows());
    B_eq << y_out - x_ref.col(0), zero_matrix_bep;

    return std::make_tuple(A_eq, B_eq);

}

Eigen::VectorXd MPC::SolveMPCProblem(Eigen::MatrixXd H, Eigen::VectorXd f,
                                    Eigen::MatrixXd A_eq, Eigen::VectorXd B_eq,
                                    Eigen::MatrixXd A_in, Eigen::VectorXd B_in) {
    // Set value for QP Matrices

    // The problem is in the form:

    // min 0.5 * x G x + g0 x
    // s.t.
    //     CE^T x + ce0 = 0             -> CE^T x = -ce0        -> Aeq X = -Beq
    //     CI^T x + ci0 >= 0            -> -CI^T x <= ci0       -> -Ain X <= Bin
        
    //  The matrix and vectors dimensions are as follows:
    //      G: n * n
    // 		g0: n
                    
    // 		CE: neq * p
    // 	    ce0: p
                    
    // 	    CI: nin * m
    //      ci0: m

    //      x: n

    // Quadratic Programming
    quadprogpp::Matrix<double> H_qp, A_eq_qp, A_in_qp;
    quadprogpp::Vector<double> f_qp, B_eq_qp, B_in_qp, x_qp;

    int n = H.rows();
    int neq = A_eq.rows();
    int nin = A_in.rows();

    H_qp.resize(n,n);
    A_eq_qp.resize(n,neq);
    A_in_qp.resize(n,nin);
    f_qp.resize(n);
    B_eq_qp.resize(neq);
    B_in_qp.resize(nin);
    x_qp.resize(n);

    Eigen::MatrixXd H_T = H.transpose();
    Eigen::MatrixXd A_eq_T = A_eq.transpose();
    Eigen::MatrixXd A_in_T = A_in.transpose();

    for (size_t i = 0; i < n; i++)
    {
        for (size_t ii = 0; ii < n; ii++)
        {H_qp[i][ii] = H(i,ii);}        
    }

    for (size_t i = 0; i < n; i++)
    {
        for (size_t ii = 0; ii < neq; ii++)
        {A_eq_qp[i][ii] = A_eq_T(i,ii);}        
    }

    for (size_t i = 0; i < n; i++)
    {
        for (size_t ii = 0; ii < nin; ii++)
        {A_in_qp[i][ii] = -A_in_T(i,ii);}        
    }

    for (size_t i = 0; i < n; i++)
    {f_qp[i] = 0;}

    for (size_t i = 0; i < neq; i++)
    {B_eq_qp[i] = -B_eq(i);}


    for (size_t i = 0; i < nin; i++)
    {B_in_qp[i] = B_in(i);}

    quadprogpp::solve_quadprog(H_qp,f_qp,A_eq_qp,B_eq_qp,A_in_qp,B_in_qp,x_qp);

    // std::cout << x_qp << std::endl;

    Eigen::VectorXd x_OL(x_qp.size());
    for (size_t i = 0; i < x_qp.size() ; i++)
    {
        x_OL(i) = x_qp[i];
    }
    // std::cout << x_OL << std::endl;

    // =====================================================
    Eigen::VectorXd state_OL_tilde(nx_*(predict_steps_+1));
    Eigen::VectorXd input_OL_tilde(nu_*predict_steps_);

    for (size_t i = 0; i < nx_*(predict_steps_+1); i++)
    {
        state_OL_tilde[i] = x_qp[i];
    }

    for (size_t i = nx_*(predict_steps_+1); i < nx_*(predict_steps_+1) + nu_*predict_steps_; i++)
    {
        input_OL_tilde(i - nx_*(predict_steps_+1)) = x_qp[i];       
    }
    
    // std::cout << input_OL_tilde << std::endl;

    // Resize
    Eigen::MatrixXd sol_x(nx_, predict_steps_+1);
    Eigen::MatrixXd sol_u(nu_, predict_steps_);

    for (size_t i = 0; i < sol_x.cols(); i++)
    {
            for (size_t ii = 0; ii < sol_x.rows(); ii++)
            {
                    sol_x(ii,i) = state_OL_tilde(i*nx_ + ii);
            }
            
    }
    // std::cout << sol_x << std::endl;

    for (size_t i = 0; i < sol_u.cols(); i++)
    {
            for (size_t ii = 0; ii < sol_u.rows(); ii++)
            {
                    sol_u(ii,i) = input_OL_tilde(i*nu_ + ii);
            }
            
    }
    // std::cout << sol_u << std::endl;

    Eigen::VectorXd new_u(nu_);
    new_u = u_ref_.col(0) + sol_u.col(0);
    // std::cout << new_u << std::endl;

    return new_u;
}

void MPC::Control(Eigen::Vector3d robot_pose)
{
    double distance, goal_check_dis;
    double lin_vel, steer_ang;

    goal_check_dis = (end_goal_ - robot_pose).norm();
    // std::cout << "goal_check_dis:\n" << goal_check_dis << std::endl; 
    Eigen::Vector2d robot_position = robot_pose.head<2>();

    for (size_t i = counter_; i < iterations_; i++)
    {
        GetHorizonTrajectory(i);
        Eigen::Vector2d mpc_goal = x_N_goal_.head<2>(); 
        distance = (robot_position - mpc_goal).norm();
        // std::cout << "current_goal_distance:" <<  distance << "\n" << std::endl;
        
        // Set Constraints in current step
        std::tie(Aeq_, Beq_) = SetEqualityConstraints(x_N_, u_N_, robot_pose);

        // Solve MPC Problems
        optimal_solution_ = SolveMPCProblem(H_, f_, Aeq_, Beq_, Ai_, Bi_);

        // lin_vel = optimal_solution_(0);
        lin_vel = std::min(optimal_solution_(0), 1.0);

        if (steer_ang > 0)
        {
            steer_ang = std::min(optimal_solution_(1), 0.69813170079);
        }
        else if (steer_ang < 0)
        {
            steer_ang = std::max(optimal_solution_(1), -0.69813170079);
        }

        // Check for step goal---------

        if (distance <= 0.25)
        {   
            counter_ = i;
            break;
        }

        else if(goal_check_dis < 1.0)
        {
            ROS_WARN("LAST POINT");
            counter_ = x_ref_.cols() -1;
            break;
        }
    }

    double distance_to_goal = (end_goal_.head<2>() - robot_position).norm();

    if (distance_to_goal < 0.5)
    {
        lin_vel = 0.9;
    }

    if (distance_to_goal < 0.1)
    {
        StopMotion(robot_pose);
    }
    else
    {
        // std::cout << counter_ << std::endl;
        Eigen::Vector2d input;
        input << lin_vel, steer_ang;
        PublishControlCommand(input);
    }
}

void MPC::StraightMotion(Eigen::Vector3d robot_pose)
{
    double step_goal_distance;               // Current Horizon goal
    double lin_vel, steer_ang;      // Robot Control Vars (v, phi)

    Eigen::Vector2d robot_position = robot_pose.head<2>();

    for (size_t i = counter_; i < iterations_; i++)
    {
        GetHorizonTrajectory(counter_);
        Eigen::Vector2d mpc_goal = x_N_goal_.head<2>();
        step_goal_distance = (robot_position - mpc_goal).norm();
        // std::cout << "current_goal_distance:" <<  step_goal_distance << "\n" << std::endl;
    
        if (step_goal_distance > 0.3)
        {
            counter_ = i;
            break;
        }
        else
        {
            counter_ = i+1;
        }
    }

    if (curve_detect[counter_] == 0)
    {
        lin_vel = 1.0;
        steer_ang = 0.0;
    }
    else
    {
        lin_vel = 0.0;
        steer_ang = 0.0;
    }

    std::cout << counter_ << "," << curve_detect[counter_] << "," << lin_vel << "," << steer_ang << std::endl;
    // std::cout << "step :" << counter_ << ", curve :" << curve_detect[counter_] << ", linear vel :" << lin_vel << ", steering angle :" << steer_ang << std::endl;
    Eigen::Vector2d input;
    input << lin_vel, steer_ang;
    PublishControlCommand(input);

}

void MPC::Control2(Eigen::Vector3d robot_pose)
{
    double step_goal_distance;      // Current Horizon goal
    double lin_vel, steer_ang;      // Robot Control Vars (v, phi)

    Eigen::Vector2d robot_position = robot_pose.head<2>();

    for (size_t i = counter_; i < iterations_; i++)
    {
        GetHorizonTrajectory(counter_);
        Eigen::Vector2d mpc_goal = x_N_goal_.head<2>();
        std::cout << "Current goal : "  << mpc_goal << ", current position : " << robot_position << std::endl;
        step_goal_distance = (robot_position - mpc_goal).norm();
        // std::cout << "current_goal_distance:" <<  step_goal_distance << "\n" << std::endl;
    
        if (step_goal_distance > 0.5)
        {
            counter_ = i;
            break;
        }
        else
        {
            counter_ = i+1;
        }
    }

    if (curve_detect[counter_] == 0)
    {
        lin_vel = 1.0;
        steer_ang = 0.0;
    }
    else
    {
        // Set Constraints in current step
        std::tie(Aeq_, Beq_) = SetEqualityConstraints(x_N_, u_N_, robot_pose);

        // Solve MPC Problems
        optimal_solution_ = SolveMPCProblem(H_, f_, Aeq_, Beq_, Ai_, Bi_);

        // lin_vel = optimal_solution_(0);
        lin_vel = std::min(optimal_solution_(0), 1.0);

        if (steer_ang > 0)
        {
            steer_ang = std::min(optimal_solution_(1), 0.639635);     // 0.69813170079     0.785398
        }
        else if (steer_ang < 0)
        {
            steer_ang = std::max(optimal_solution_(1), -0.639635);    // 0.69813170079     0.785398
        }
        // steer_ang = -steer_ang;
    }

    std::cout << "step :" << counter_ << ", curve :" << curve_detect[counter_] << ", linear vel :" << lin_vel << ", steering angle :" << steer_ang << std::endl;
    Eigen::Vector2d input;
    input << lin_vel, steer_ang;
    PublishControlCommand(input);

}

void MPC::Control3(Eigen::Vector3d robot_pose)
{
    double step_goal_distance;      // Current Horizon goal
    double lin_vel, steer_ang;      // Robot Control Vars (v, phi)

    Eigen::Vector2d robot_position = robot_pose.head<2>();

    for (size_t i = counter_; i < iterations_; i++)
    {
        GetHorizonTrajectory(counter_);
        Eigen::Vector2d mpc_goal = x_N_goal_.head<2>();
        std::cout << "Current goal : "  << mpc_goal << ", current position : " << robot_position << std::endl;
        step_goal_distance = (robot_position - mpc_goal).norm();

        if (step_goal_distance > 0.4)
        {
            counter_ = i;
            break;
        }
        else
        {
            counter_ = i+1;
        }
    }

    // Straight Motion
    if (curve_detect[counter_] == 0)
    {
        // double y_des = x_N_goal_(1);
        // double curr_y = robot_position(1);
        // double e_lat = curr_y - y_des;
        // std::cout << "y_ref, y_pos, e_lat :"  << y_des << "," << curr_y << "," << e_lat << std::endl;
        // Kp = 0.9;

        // if (abs(e_lat) > 0.)
        // {
        //     lin_vel = 1.0;
        //     steer_ang = Kp * e_lat;
        //     std::cout << "input : " << lin_vel << "," << steer_ang << std::endl; 
        // }
        // else
        // {
        //     lin_vel = 1.0;
        //     steer_ang = 0.0;
        // }
        lin_vel = 1.0;
        steer_ang = 0.0;
    }
    // Turning Motion
    else
    {
        // Set Constraints in current step
        std::tie(Aeq_, Beq_) = SetEqualityConstraints(x_N_, u_N_, robot_pose);

        // Solve MPC Problems
        optimal_solution_ = SolveMPCProblem(H_, f_, Aeq_, Beq_, Ai_, Bi_);

        lin_vel = optimal_solution_(0);
        // lin_vel = std::min(optimal_solution_(0), 1.0);

        if (steer_ang > 0)
        {
            steer_ang = std::min(optimal_solution_(1), 0.639635);     // 0.69813170079     0.785398
        }
        else if (steer_ang < 0)
        {
            steer_ang = std::max(optimal_solution_(1), -0.639635);    // 0.69813170079     0.785398
        }
        steer_ang = -steer_ang;
    }

    std::cout << "step :" << counter_ << ", curve :" << curve_detect[counter_] << ", linear vel :" << lin_vel << ", steering angle :" << steer_ang << std::endl;
    Eigen::Vector2d input;
    input << lin_vel, steer_ang;
    PublishControlCommand(input);

}

void MPC::StraightMotionWithKp(Eigen::Vector3d robot_pose)
{
    double step_goal_distance;               // Current Horizon goal
    double lin_vel, steer_ang;      // Robot Control Vars (v, phi)

    Eigen::Vector2d robot_position = robot_pose.head<2>();

    for (size_t i = counter_; i < iterations_; i++)
    {
        GetHorizonTrajectory(counter_);
        Eigen::Vector2d mpc_goal = x_N_goal_.head<2>();
        step_goal_distance = (robot_position - mpc_goal).norm();
        // std::cout << "current_goal_distance:" <<  step_goal_distance << "\n" << std::endl;
    
        if (step_goal_distance > 0.3)
        {
            counter_ = i;
            break;
        }
        else
        {
            counter_ = i+1;
        }
    }

    if (curve_detect[counter_] == 0)
    {
        double y_des = x_N_goal_(1);
        double curr_y = robot_position(1);
        double e_lat = curr_y - y_des;
        std::cout << "y_ref, y_pos, e_lat :"  << y_des << "," << curr_y << "," << e_lat << std::endl;

        if (abs(e_lat) > 0.1)
        {
            double curr_orient = x_N_goal_(2);
            Kp = 2;
            if (curr_orient == 0)
            {
                lin_vel = 1;
                steer_ang = -Kp * e_lat;
                std::cout << "input : " << lin_vel << "," << steer_ang << std::endl; 
            }
            else
            {
                lin_vel = 1;
                steer_ang = Kp * e_lat;
                std::cout << "input : " << lin_vel << "," << steer_ang << std::endl; 
            }
        }

        else if (abs(e_lat) < 0.1 && abs(e_lat) > 0.05 )
        {
            double curr_orient = x_N_goal_(2);
            Kp = 0.8;
            if (curr_orient == 0)
            {
                lin_vel = 0.5;
                steer_ang = -Kp * e_lat;
                std::cout << "input : " << lin_vel << "," << steer_ang << std::endl; 
            }
            else
            {
                lin_vel = 0.5;
                steer_ang = Kp * e_lat;
                std::cout << "input : " << lin_vel << "," << steer_ang << std::endl; 
            }
        }
        
        else
        {
            lin_vel = 1.0;
            steer_ang = 0.0;
        }
    }
    else
    {
        lin_vel = 0.0;
        steer_ang = 0.0;
    }

    std::cout << counter_ << "," << curve_detect[counter_] << "," << lin_vel << "," << steer_ang << std::endl;
    // std::cout << "step :" << counter_ << ", curve :" << curve_detect[counter_] << ", linear vel :" << lin_vel << ", steering angle :" << steer_ang << std::endl;
    Eigen::Vector2d input;
    input << lin_vel, steer_ang;
    PublishControlCommand(input);
}

void MPC::StraightMotionWithStanley(Eigen::Vector3d robot_pose)
{
    double step_goal_distance;               // Current Horizon goal
    double lin_vel, steer_ang;      // Robot Control Vars (v, phi)

    Eigen::Vector2d robot_position = robot_pose.head<2>();

    for (size_t i = counter_; i < iterations_; i++)
    {
        GetHorizonTrajectory(counter_);
        Eigen::Vector2d mpc_goal = x_N_goal_.head<2>();
        step_goal_distance = (robot_position - mpc_goal).norm();
    
        if (step_goal_distance > 0.3)
        {
            counter_ = i;
            break;
        }
        else
        {
            counter_ = i+1;
        }
    }

    if (curve_detect[counter_] == 0)
    {
        
        double pos_x = robot_pose(0);
        double pos_y = robot_pose(1);
        double theta = robot_pose(2);
        double a,b,c;
        double e, phi, delta;

        std::cout << pos_x << "," << pos_y << "," << theta << std::endl;

        if (x_N_goal_(2) == 0)
        {
            a = 0;
            b = -1;
            c = 0;
            e = (a*pos_x + b*pos_y + c)/sqrt(a*a + b*b);
            Kp = 0.9;
            phi = atan2(b,-a)-theta;
            delta = phi + atan2(1,Kp*e);
        
            lin_vel = 1;
            steer_ang = delta;
        }
        if (x_N_goal_(2) == 3)
        {
            a = 0;
            b = -1;
            c = -3;
            e = (a*pos_x + b*pos_y + c)/sqrt(a*a + b*b);
            Kp = 0.9;
            phi = atan2(b,-a)-theta;
            delta = phi + atan2(1,Kp*e);
            lin_vel = 1;
            steer_ang = delta;
        }
    }
    else
    {
        lin_vel = 0.0;
        steer_ang = 0.0;
    }

    std::cout << counter_ << "," << curve_detect[counter_] << "," << lin_vel << "," << steer_ang << std::endl;
    double distance_to_goal = (end_goal_.head<2>() - robot_position).norm();
    if (distance_to_goal < 0.5)
    {
        lin_vel = 0.9;
    }

    if (distance_to_goal < 0.1)
    {
        StopMotion(robot_pose);
    }
    else
    {
        Eigen::Vector2d input;
        input << lin_vel, steer_ang;
        PublishControlCommand(input);
    }

}

void MPC::Control4(Eigen::Vector3d front_wheel_pose)
{
    double step_goal_distance;      // Current Horizon goal
    double lin_vel, steer_ang;      // Robot Control Vars (v, phi)

    Eigen::Vector2d robot_position = front_wheel_pose.head<2>();

    for (size_t i = counter_; i < iterations_; i++)
    {
        GetHorizonTrajectory(counter_);
        Eigen::Vector2d mpc_goal = x_N_goal_.head<2>();
        // std::cout << "Current goal : "  << mpc_goal << ", current position : " << robot_position << std::endl;
        step_goal_distance = (robot_position - mpc_goal).norm();

        if (step_goal_distance > 0.4)
        {
            counter_ = i;
            break;
        }
        else
        {
            counter_ = i+1;
        }
    }

    // Straight Motion
    if (curve_detect[counter_] == 0)
    {
        lin_vel = 1.0;
        steer_ang = 0.0;
    }
    // Turning Motion
    else
    {
        // Set Constraints in current step
        std::tie(Aeq_, Beq_) = SetEqualityConstraints(x_N_, u_N_, front_wheel_pose);

        // Solve MPC Problems
        optimal_solution_ = SolveMPCProblem(H_, f_, Aeq_, Beq_, Ai_, Bi_);

        lin_vel = optimal_solution_(0);
        // lin_vel = std::min(optimal_solution_(0), 1.0);

        if (steer_ang > 0)
        {
            steer_ang = std::min(optimal_solution_(1), 0.69813170079);     // 0.69813170079     0.785398       0.639635
        }
        else if (steer_ang < 0)
        {
            steer_ang = std::max(optimal_solution_(1), -0.69813170079);    // 0.69813170079     0.785398       0.639635
        }
        steer_ang = -steer_ang;
    }

    std::cout << "step :" << counter_ << ", curve :" << curve_detect[counter_] << ", linear vel :" << lin_vel << ", steering angle :" << steer_ang << std::endl;
    
    double distance_to_goal = (end_goal_.head<2>() - robot_position).norm();
    if (distance_to_goal < 0.3)
    {
        lin_vel = 0.9;
    }

    if (distance_to_goal < 0.1)
    {
        StopMotion(front_wheel_pose);
    }
    else
    {
        Eigen::Vector2d input;
        input << lin_vel, steer_ang;
        PublishControlCommand(input);
    }

}