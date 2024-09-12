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

#include "model_predictive_contouring_control/params.h"
namespace mpcc{
    
Param::Param(){
    std::cout << "Default initialization of model params" << std::endl;
}

Param::Param(std::string file){
    /////////////////////////////////////////////////////
    // Loading Model and Constraint Parameters //////////
    /////////////////////////////////////////////////////
    // std::cout << "model" << std::endl;

    std::ifstream iModel(file);
    json jsonModel;
    iModel >> jsonModel;
    // Model Parameters
    length_base 	= jsonModel["length_base"];

    car_l = jsonModel["car_l"];
    car_w = jsonModel["car_w"];
    
    g = jsonModel["g"];
    //Constraint Parameters
    r_in = jsonModel["R_in"];
    r_out = jsonModel["R_out"];

    max_dist_proj = jsonModel["max_dist_proj"];

    e_long = jsonModel["E_long"];
    e_eps = jsonModel["E_eps"];

    // initial warm start and trust region (model dependent)
    initial_velocity = jsonModel["initial_velocity"];
    s_trust_region = jsonModel["s_trust_region"];

    vx_zero = jsonModel["vx_zero"];
}

CostParam::CostParam(){
    std::cout << "Default initialization of cost" << std::endl;
}

CostParam::CostParam(std::string file){
    /////////////////////////////////////////////////////
    // Loading Cost Parameters //////////////////////////
    /////////////////////////////////////////////////////
    // std::cout << "cost" << std::endl;

    std::ifstream iCost(file);
    json jsonCost;
    iCost >> jsonCost;

    q_c = jsonCost["qC"];
    q_l = jsonCost["qL"];
    q_vs = jsonCost["qVs"];

    q_mu = jsonCost["qMu"];

    r_V = jsonCost["rV"];
    r_delta = jsonCost["rDelta"];
    r_vs = jsonCost["rVs"];

    r_dV = jsonCost["rdV"];
    r_dDelta = jsonCost["rdDelta"];
    r_dVs = jsonCost["rdVs"];

    q_c_N_mult = jsonCost["qCNmult"];
    q_r_N_mult = jsonCost["qRNmult"];

    sc_quad_track = jsonCost["sc_quad_track"];

    sc_lin_track = jsonCost["sc_lin_track"];
}

BoundsParam::BoundsParam() {
    std::cout << "Default initialization of bounds" << std::endl;
}

BoundsParam::BoundsParam(std::string file) {

    /////////////////////////////////////////////////////
    // Loading Cost Parameters //////////////////////////
    /////////////////////////////////////////////////////
    // std::cout << "bounds" << std::endl;

    std::ifstream iBounds(file);
    json jsonBounds;
    iBounds >> jsonBounds;

    lower_state_bounds.X_l = jsonBounds["Xl"];
    lower_state_bounds.Y_l = jsonBounds["Yl"];
    lower_state_bounds.phi_l = jsonBounds["phil"];
    lower_state_bounds.s_l = jsonBounds["sl"];
    lower_state_bounds.v_l = jsonBounds["vl"];
    lower_state_bounds.delta_l = jsonBounds["deltal"];
    lower_state_bounds.vs_l = jsonBounds["vsl"];

    upper_state_bounds.X_u = jsonBounds["Xu"];
    upper_state_bounds.Y_u = jsonBounds["Yu"];
    upper_state_bounds.phi_u = jsonBounds["phiu"];
    upper_state_bounds.s_u = jsonBounds["su"];
    upper_state_bounds.v_u = jsonBounds["vu"];
    upper_state_bounds.delta_u = jsonBounds["deltau"];
    upper_state_bounds.vs_u = jsonBounds["vsu"];

    lower_input_bounds.dV_l = jsonBounds["dVl"];
    lower_input_bounds.dDelta_l = jsonBounds["dDeltal"];
    lower_input_bounds.dVs_l = jsonBounds["dVsl"];

    upper_input_bounds.dV_u = jsonBounds["dVu"];
    upper_input_bounds.dDelta_u = jsonBounds["dDeltau"];
    upper_input_bounds.dVs_u = jsonBounds["dVsu"];
}

NormalizationParam::NormalizationParam(){
    std::cout << "Default initialization of normalization" << std::endl;
}

NormalizationParam::NormalizationParam(std::string file)
{
    /////////////////////////////////////////////////////
    // Loading Normalization Parameters /////////////////
    /////////////////////////////////////////////////////
    // std::cout << "norm" << std::endl;

    std::ifstream iNorm(file);
    json jsonNorm;
    iNorm >> jsonNorm;

    T_x.setIdentity();
    T_x(si_index.X,si_index.X) = jsonNorm["X"];
    T_x(si_index.Y,si_index.Y) = jsonNorm["Y"];
    T_x(si_index.phi,si_index.phi) = jsonNorm["phi"];
    T_x(si_index.s,si_index.s) = jsonNorm["s"];
    T_x(si_index.v,si_index.v) = jsonNorm["v"];
    T_x(si_index.delta,si_index.delta) = jsonNorm["delta"];
    T_x(si_index.vs,si_index.vs) = jsonNorm["vs"];


    T_x_inv.setIdentity();
    for(int i = 0;i<NX;i++)
    {
        T_x_inv(i,i) = 1.0/T_x(i,i);
    }

    T_u.setIdentity();
    T_u(si_index.dV,si_index.dV) = jsonNorm["dV"];
    T_u(si_index.dDelta,si_index.dDelta) = jsonNorm["dDelta"];
    T_u(si_index.dVs,si_index.dVs) = jsonNorm["dVs"];

    T_u_inv.setIdentity();
    for(int i = 0;i<NU;i++)
    {
        T_u_inv(i,i) = 1.0/T_u(i,i);
    }

    T_s.setIdentity();
    T_s_inv.setIdentity();
}

}
