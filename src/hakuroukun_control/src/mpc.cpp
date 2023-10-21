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
MPC::MPC(ros::NodeHandle nh, ros::NodeHandle private_nh, double sampling_time, sdv_msgs::Trajectory &traj)
{
    // Set dt
    sampling_time_ = sampling_time;

    // MPC Parameters
    SetMPCParameters();

    // Predict horizon trajectory
    trajectory_.header = traj.header;
    trajectory_.points = std::vector<sdv_msgs::TrajectoryPoint>(traj.points.begin(), traj.points.end());
    ReadTrajectory();

    // std::cout << trajectory_.points.size() << std::endl;
    // std::cout << iterations_ << std::endl;

    // MPC Matrices
    std::tie(H_, f_) = SetStageCost();
    std::tie(Ai_, Bi_) = SetInequalityConstraints();
    
}

void MPC::ReadTrajectory()
{
    x_ref_ = Eigen::MatrixXd(nx_, trajectory_.points.size());
    u_ref_ = Eigen::MatrixXd(nu_, trajectory_.points.size());
    end_goal_ = Eigen::VectorXd(nx_);

    for(int i=0; i<trajectory_.points.size(); i++)
    {
        x_ref_(0,i) = trajectory_.points.at(i).x;
        x_ref_(1,i) = trajectory_.points.at(i).y;
        x_ref_(2,i) = trajectory_.points.at(i).heading;
        
        u_ref_(0,i) = trajectory_.points.at(i).velocity_mps;
        u_ref_(1,i) = trajectory_.points.at(i).steering_angle;
    }
    // end_goal_ = x_ref_.block(0, x_ref_.cols(), 3, 1);

    iterations_ = trajectory_.points.size() - predict_steps_;

}

void MPC::GetHorizonTrajectory(double current_step)
{
    x_N_ = Eigen::MatrixXd(nx_, predict_steps_+1);
    u_N_ = Eigen::MatrixXd(nu_, predict_steps_+1);

    for (size_t i = 0; i <= predict_steps_; i++)
    {
        x_N_(0,i) = x_ref_(0, current_step + i);
        x_N_(1,i) = x_ref_(1, current_step + i);
        x_N_(2,i) = x_ref_(2, current_step + i);
        
        u_N_(0,i) = u_ref_(0, current_step + i);
        u_N_(1,i) = u_ref_(1, current_step + i);     
    }
}


// ========================================================
// PRIVATE FUNCTION
// ========================================================
void MPC::SetMPCParameters()
{
    Model model(sampling_time_);
    nx_ = model.nx;
    nu_ = model.nu;
    predict_steps_ = 10;
}


// ========================================================
// MPC FUNCTIONS
// ========================================================
std::tuple<Eigen::MatrixXd, Eigen::VectorXd> MPC::SetStageCost(){

    Eigen::MatrixXd Q(nx_,nx_);
    Eigen::MatrixXd R(nu_,nu_);
    Q   <<  200,  0.0,    0.0,
            0.0,  200,    0.0,
            0.0,  0.0, 0.0001;

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
    Eigen::MatrixXd S_(2*nu_, 2*nu_);
    S_<< 1.0, 0.0 ,-1.0, 0.0,
        -1.0, 0.0 , 1.0, 0.0,
         0.0, 1.0 , 0.0,-1.0,
         0.0,-1.0 , 0.0, 1.0;
         
    Eigen::MatrixXd K_(2*nu_,1);
    K_ << 1.5, 0.5, M_PI, 0.0;

    Eigen::MatrixXd A_u(S_.rows()*(predict_steps_-1), ((predict_steps_)*nu_));
    Eigen::MatrixXd B_u(K_.rows()*(predict_steps_-1), 1);

    for (size_t i = 0; i < predict_steps_-1; i++)
    {
        Eigen::MatrixXd A_u_k(S_.rows(), ((predict_steps_)*nu_));
        Eigen::MatrixXd zero_matrix_1 = Eigen::MatrixXd::Zero(S_.rows(),nu_*i);
        Eigen::MatrixXd zero_matrix_2 = Eigen::MatrixXd::Zero(S_.rows(), ((predict_steps_)*nu_ - S_.cols() - zero_matrix_1.cols()));
        A_u_k << zero_matrix_1, S_, zero_matrix_2;

        A_u.row(S_.rows()*i)   = A_u_k.row(0);
        A_u.row(S_.rows()*i+1) = A_u_k.row(1);
        A_u.row(S_.rows()*i+2) = A_u_k.row(2);
        A_u.row(S_.rows()*i+3) = A_u_k.row(3);
    
        B_u.row(K_.rows()*i)   = K_.row(0);
        B_u.row(K_.rows()*i+1) = K_.row(1);
        B_u.row(K_.rows()*i+2) = K_.row(2);
        B_u.row(K_.rows()*i+3) = K_.row(3);
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

    // Quadratic Programming
    quadprogpp::Matrix<double> H_qp, A_eq_qp, A_in_qp;
    quadprogpp::Vector<double> f_qp, B_eq_qp, B_in_qp, x_qp;

    int n = H.rows();
    int neq = A_eq.rows();
    int nin = A_in.rows();
    double sum = 0.0;

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

    // Set value for QP Matrices

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
    // {B_eq_qp[i] = B_eq(i);}


    for (size_t i = 0; i < nin; i++)
    // {B_in_qp[i] = -B_in(i);}
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
    std::cout << sol_u << std::endl;

    Eigen::VectorXd new_u(nu_);
    new_u = u_ref_.col(0) + sol_u.col(0);
    std::cout << new_u << std::endl;

    return new_u;
}








