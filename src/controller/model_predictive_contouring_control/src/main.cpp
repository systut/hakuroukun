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

int main() {

    using namespace mpcc;
    std::ifstream iConfig("src/controller/model_predictive_contouring_control/config/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    // std::cout << testSpline() << std::endl;
    // std::cout << testArcLengthSpline(json_paths) << std::endl;

    // std::cout << testIntegrator(json_paths) << std::endl;
    // std::cout << testLinModel(json_paths) << std::endl;

    // std::cout << testCost(json_paths) << std::endl;

    // Test--------------------------------------------------------------
    // int return_flag;                                                                
                                                                                    
    // return_flag = testSpline();                                                     
    // std::cout << " Result of testSpline(): " << return_flag << std::endl;           
                                                                                    
    // return_flag = testArcLengthSpline(json_paths);                                  
    // std::cout << " Result of testArcLengthSpline(): " << return_flag << std::endl;  
                                                                                    
    // return_flag = testIntegrator(json_paths);                                       
    // std::cout << " Result of testIntegrator(): " <<  return_flag << std::endl;      
                                                                                    
    // return_flag = testLinModel(json_paths);                                         
    // std::cout << " Result of testLinModel(): " << return_flag << std::endl;         
                                                                                    
    // return_flag = testTrackConstraint(json_paths);                                  
    // std::cout << " Result of testTrackConstraint(): " << return_flag << std::endl;  
                                                                                    
    // return_flag = testCost(json_paths);                                             
    // std::cout << " Result of cost(): " << return_flag << std::endl;                 
                                                                                    
    // ------------------------------------------------------------------           

    Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
    Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);

    Track track = Track(json_paths.track_path);
    TrackPos track_xy = track.getTrack();

    std::list<MPCReturn> log;
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    mpc.setTrack(track_xy.X,track_xy.Y);
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    State x0 = {track_xy.X(0),track_xy.Y(0),phi_0,0,jsonConfig["v0"],0,jsonConfig["v0"]};
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {
        MPCReturn mpc_sol = mpc.runMPC(x0);
        x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
        log.push_back(mpc_sol);
    }
    plotter.plotRun(log,track_xy);
    plotter.plotSim(log,track_xy);

    double mean_time = 0.0;
    double max_time = 0.0;
    for(MPCReturn log_i : log)
    {
        mean_time += log_i.time_total;
        if(log_i.time_total > max_time)
            max_time = log_i.time_total;
    }
    std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time << std::endl;
    return 0;
}


