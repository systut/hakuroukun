/*
 * ===============================================================================
 * model.cpp
 * Author: Dinh Ngoc Duc
 * Date: 13.10.23
 * Email: duc.dn.st@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This is the main node for the dynamic of Hakuroukun. 
 * ===============================================================================
 */

#include "../include/model.h"

Model::Model(double sampling_time)
{
    init();
    sampling_time_ = sampling_time;

}

// ========================================================
// PUBLIC FUNCTION
// ========================================================
Eigen::VectorXd Model::DynamicFunction(Eigen::Vector3d x, Eigen::Vector2d u)
{
    Eigen::Vector3d f;
    Eigen::Vector3d dfdt_;

    dfdt_<< u(0) * cos(u(1)) * cos(x(2)),
            u(0) * cos(u(1)) * sin(x(2)),
            u(0) * sin(u(1)) / length_base;

    f = x + dfdt_*sampling_time_;

    return f;
}


Eigen::MatrixXd Model::SystemMatrix(Eigen::Vector3d x, Eigen::Vector2d u)
{
    Eigen::MatrixXd A(nx, nx);
    Eigen::MatrixXd A_u(nx, nx); 

    A_u <<  0, 0, -u(0)* cos(u(1)) * sin(x(2)),
            0, 0,  u(0)* cos(u(1)) * cos(x(2)),
            0, 0, 0;

    A = Eigen::MatrixXd::Identity(nx,nx) + A_u * sampling_time_;

    return A;
}

Eigen::MatrixXd Model::ControlMatrix(Eigen::Vector3d x, Eigen::Vector2d u)
{
    Eigen::MatrixXd B(nx, nu);
    Eigen::MatrixXd B_u(nx, nu);

    B_u <<  cos(u(1)) * cos(x(2))  , -u(0) * sin(u(1)) * cos(x(2)),
            cos(u(1)) * sin(x(2))  , -u(0) * sin(u(1)) * sin(x(2)),
            sin(u(1)) / length_base,  u(0) * cos(u(1)) / length_base; 

    B = B_u * sampling_time_;

    return B;
}

// ========================================================
// PRIVATE FUNCTION
// ========================================================

void Model::init()
{
    nx = 3;
    nu = 2;
    length_base = 0.95;

    LIN_VEL_MAX = 1.5;
    LIN_VEL_MAX = 0.5;
    ANG_VEL_MAX = M_PI;
    ANG_VEL_MAX = 0;

}

