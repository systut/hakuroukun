// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "model_predictive_contouring_control/model.h"
namespace mpcc{
Model::Model()
:Ts_(1.0)
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Model::Model(double Ts,const PathToJson &path)
:Ts_(Ts),param_(Param(path.param_path))
{
}

StateVector Model::getF(const State &x,const Input &u) const
{
    const double phi = x.phi;
    const double v = x.v;
    const double delta = x.delta;
    const double vs = x.vs;

    const double dV = u.dV;
    const double dDelta = u.dDelta;
    const double dVs = u.dVs;

    StateVector f;
    f(0) = v*std::cos(delta)*std::cos(phi);
    f(1) = v*std::cos(delta)*std::sin(phi);
    f(2) = v*std::sin(delta) / param_.length_base;
    f(3) = vs;
    f(4) = dV;
    f(5) = dDelta;
    f(6) = dVs;

    // f(0) = v*std::cos(delta+phi);
    // f(1) = v*std::sin(delta+phi);
    // f(2) = v*std::sin(delta) / param_.length_base;
    // f(3) = vs;
    // f(4) = dV;
    // f(5) = dDelta;
    // f(6) = dVs;

    return f;
}

LinModelMatrix Model::getModelJacobian(const State &x, const Input &u) const
{
    // compute jacobian of the model
    // state values

    const double phi = x.phi;
    const double v = x.v;
    const double delta = x.delta;

    const double dV = u.dV;
    const double dDelta = u.dDelta;
    const double dVs = u.dVs;

//    LinModelMatrix lin_model_c;
    A_MPC A_c = A_MPC::Zero();
    B_MPC B_c = B_MPC::Zero();
    g_MPC g_c = g_MPC::Zero();

    const StateVector f = getF(x,u);

    // f1 = v*std::cos(delta)*std::cos(phi);
    const double df1_dx = 0.0;
    const double df1_dy = 0.0;
    const double df1_dphi = -v*std::cos(delta)*std::sin(phi);
    const double df1_ds = 0.0;
    const double df1_dV = std::cos(delta)*std::cos(phi);
    const double df1_dDelta = -v*std::sin(delta)*std::cos(phi);
    const double df1_dVs = 0.0; 

    // f2 = v*std::cos(delta)*std::sin(phi);
    const double df2_dx = 0.0;
    const double df2_dy = 0.0;
    const double df2_dphi = v*std::cos(delta)*std::cos(phi);
    const double df2_ds = 0.0;
    const double df2_dV = std::cos(delta)*std::sin(phi);
    const double df2_dDelta = -v*std::sin(delta)*std::sin(phi);
    const double df2_dVs = 0.0; 

    // f3 = v*std::sin(delta) / length_base;
    const double df3_dx = 0.0;
    const double df3_dy = 0.0;
    const double df3_dphi = 0.0;
    const double df3_ds = 0.0;
    const double df3_dV = std::sin(delta) / param_.length_base;
    const double df3_dDelta = v*std::cos(delta) / param_.length_base;
    const double df3_dVs = 0.0; 
    
    // f4 = vs;
    const double df4_dx = 0.0;
    const double df4_dy = 0.0;
    const double df4_dphi = 0.0;
    const double df4_ds = 0.0;
    const double df4_dV = 0.0;
    const double df4_dDelta = 0.0;
    const double df4_dVs = 1.0; 

    // f5 = dV;
    const double df5_dx = 0.0;
    const double df5_dy = 0.0;
    const double df5_dphi = 0.0;
    const double df5_ds = 0.0;
    const double df5_dV = 0.0;
    const double df5_dDelta = 0.0;
    const double df5_dVs = 0.0;

    // f6 = dDelta;
    const double df6_dx = 0.0;
    const double df6_dy = 0.0;
    const double df6_dphi = 0.0;
    const double df6_ds = 0.0;
    const double df6_dV = 0.0;
    const double df6_dDelta = 0.0;
    const double df6_dVs = 0.0;
    
    // f7 = dVs;
    const double df7_dx = 0.0;
    const double df7_dy = 0.0;
    const double df7_dphi = 0.0;
    const double df7_ds = 0.0;
    const double df7_dV = 0.0;
    const double df7_dDelta = 0.0;
    const double df7_dVs = 0.0;

    // // ==============================
    // // f1 = v*std::cos(delta+phi);
    // const double df1_dx = 0.0;
    // const double df1_dy = 0.0;
    // const double df1_dphi = -v*std::sin(delta+phi);
    // const double df1_ds = 0.0;
    // const double df1_dV = std::cos(delta+phi);
    // const double df1_dDelta = -v*std::sin(delta+phi);
    // const double df1_dVs = 0.0; 

    // // f2 = v*std::sin(delta+phi);
    // const double df2_dx = 0.0;
    // const double df2_dy = 0.0;
    // const double df2_dphi = v*std::cos(delta+phi);
    // const double df2_ds = 0.0;
    // const double df2_dV = std::sin(delta+phi);
    // const double df2_dDelta = v*std::cos(delta+phi);
    // const double df2_dVs = 0.0; 

    // // f3 = v*std::sin(delta) / length_base;
    // const double df3_dx = 0.0;
    // const double df3_dy = 0.0;
    // const double df3_dphi = 0.0;
    // const double df3_ds = 0.0;
    // const double df3_dV = std::sin(delta) / param_.length_base;
    // const double df3_dDelta = v*std::cos(delta) / param_.length_base;
    // const double df3_dVs = 0.0; 
    
    // // f4 = vs;
    // const double df4_dx = 0.0;
    // const double df4_dy = 0.0;
    // const double df4_dphi = 0.0;
    // const double df4_ds = 0.0;
    // const double df4_dV = 0.0;
    // const double df4_dDelta = 0.0;
    // const double df4_dVs = 1.0; 

    // // f5 = dV;
    // const double df5_dx = 0.0;
    // const double df5_dy = 0.0;
    // const double df5_dphi = 0.0;
    // const double df5_ds = 0.0;
    // const double df5_dV = 0.0;
    // const double df5_dDelta = 0.0;
    // const double df5_dVs = 0.0;

    // // f6 = dDelta;
    // const double df6_dx = 0.0;
    // const double df6_dy = 0.0;
    // const double df6_dphi = 0.0;
    // const double df6_ds = 0.0;
    // const double df6_dV = 0.0;
    // const double df6_dDelta = 0.0;
    // const double df6_dVs = 0.0;
    
    // // f7 = dVs;
    // const double df7_dx = 0.0;
    // const double df7_dy = 0.0;
    // const double df7_dphi = 0.0;
    // const double df7_ds = 0.0;
    // const double df7_dV = 0.0;
    // const double df7_dDelta = 0.0;
    // const double df7_dVs = 0.0;

    // Jacobians
    // Matrix A

    A_c(0,0) = df1_dx;
    A_c(1,0) = df2_dx;
    A_c(2,0) = df3_dx;
    A_c(3,0) = df4_dx;
    A_c(4,0) = df5_dx;
    A_c(5,0) = df6_dx;
    A_c(6,0) = df7_dx;


    A_c(0,1) = df1_dy;
    A_c(1,1) = df2_dy;
    A_c(2,1) = df3_dy;
    A_c(3,1) = df4_dy;
    A_c(4,1) = df5_dy;
    A_c(5,1) = df6_dy;
    A_c(6,1) = df7_dy;

    A_c(0,2) = df1_dphi;
    A_c(1,2) = df2_dphi;
    A_c(2,2) = df3_dphi;
    A_c(3,2) = df4_dphi;
    A_c(4,2) = df5_dphi;
    A_c(5,2) = df6_dphi;
    A_c(6,2) = df7_dphi;

    A_c(0,3) = df1_ds;
    A_c(1,3) = df2_ds;
    A_c(2,3) = df3_ds;
    A_c(3,3) = df4_ds;
    A_c(4,3) = df5_ds;
    A_c(5,3) = df6_ds;
    A_c(6,3) = df7_ds;

    A_c(0,4) = df1_dV;
    A_c(1,4) = df2_dV;
    A_c(2,4) = df3_dV;
    A_c(3,4) = df4_dV;
    A_c(4,4) = df5_dV;
    A_c(5,4) = df6_dV;
    A_c(6,4) = df7_dV;

    A_c(0,5) = df1_dDelta;
    A_c(1,5) = df2_dDelta;
    A_c(2,5) = df3_dDelta;
    A_c(3,5) = df4_dDelta;
    A_c(4,5) = df5_dDelta;
    A_c(5,5) = df6_dDelta;
    A_c(6,5) = df7_dDelta;

    A_c(0,6) = df1_dVs;
    A_c(1,6) = df2_dVs;
    A_c(2,6) = df3_dVs;
    A_c(3,6) = df4_dVs;
    A_c(4,6) = df5_dVs;
    A_c(5,6) = df6_dVs;
    A_c(6,6) = df7_dVs;

    // Matrix B
    // Column 1
    B_c(0,0) = 0.0;
    B_c(1,0) = 0.0;
    B_c(2,0) = 0.0;
    B_c(3,0) = 0.0;
    B_c(4,0) = 1.0;
    B_c(5,0) = 0.0;
    B_c(6,0) = 0.0;

    // Column 2
    B_c(0,1) = 0.0;
    B_c(1,1) = 0.0;
    B_c(2,1) = 0.0;
    B_c(3,1) = 0.0;
    B_c(4,1) = 0.0;
    B_c(5,1) = 1.0;
    B_c(6,1) = 0.0;
    
    // Column 3
    B_c(0,2) = 0.0;
    B_c(1,2) = 0.0;
    B_c(2,2) = 0.0;
    B_c(3,2) = 0.0;
    B_c(4,2) = 0.0;
    B_c(5,2) = 0.0;
    B_c(6,2) = 1.0;

    //zero order term
    g_c = f - A_c*stateToVector(x) - B_c*inputToVector(u);

    return {A_c,B_c,g_c};
}

LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c) const
{
    // disctetize the continuous time linear model \dot x = A x + B u + g using ZHO
    Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp = Eigen::Matrix<double,NX+NU+1,NX+NU+1>::Zero();
    // building matrix necessary for expm
    // temp = Ts*[A,B,g;zeros]
    temp.block<NX,NX>(0,0) = lin_model_c.A;
    temp.block<NX,NU>(0,NX) = lin_model_c.B;
    temp.block<NX,1>(0,NX+NU) = lin_model_c.g;
    temp = temp*Ts_;
    // take the matrix exponential of temp
    const Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp_res = temp.exp();
    // extract dynamics out of big matrix
    // x_{k+1} = Ad x_k + Bd u_k + gd
    //temp_res = [Ad,Bd,gd;zeros]
    const A_MPC A_d = temp_res.block<NX,NX>(0,0);
    const B_MPC B_d = temp_res.block<NX,NU>(0,NX);
    const g_MPC g_d = temp_res.block<NX,1>(0,NX+NU);

    return {A_d,B_d,g_d};
}

//LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c) const
//{
//    // disctetize the continuous time linear model \dot x = A x + B u + g using ZHO
//    Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp = Eigen::Matrix<double,NX+NU+1,NX+NU+1>::Zero();
//    // building matrix necessary for expm
//    // temp = Ts*[A,B,g;zeros]
//    temp.block<NX,NX>(0,0) = lin_model_c.A;
//    temp.block<NX,NU>(0,NX) = lin_model_c.B;
//    temp.block<NX,1>(0,NX+NU) = lin_model_c.g;
//    temp = temp*TS;
//    Eigen::Matrix<double,NX+NU+1,NX+NU+1> eye;
//    eye.setIdentity();
//    const Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp_mult = temp * temp;
//
//    const Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp_res = eye + temp + 1./2.0 * temp_mult + 1./6.0 * temp_mult * temp;
//
//    // x_{k+1} = Ad x_k + Bd u_k + gd
//    const A_MPC A_d = temp_res.block<NX,NX>(0,0);
//    const B_MPC B_d = temp_res.block<NX,NU>(0,NX);
//    const g_MPC g_d = temp_res.block<NX,1>(0,NX+NU);
//
//    return {A_d,B_d,g_d};
//
//}

LinModelMatrix Model::getLinModel(const State &x, const Input &u) const
{
    // compute linearized and discretized model
    const LinModelMatrix lin_model_c = getModelJacobian(x,u);
    // discretize the system
    return discretizeModel(lin_model_c);
}
}