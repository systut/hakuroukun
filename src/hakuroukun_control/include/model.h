#ifdef MODEL_H
#define MODEL_H

#endif // MODEL_H

#include <cmath>
#include <algorithm>
#include <mutex>
#include <vector>
#include <string>
#include <fstream>
#include <time.h>
#include <ctime>

#include <eigen3/Eigen/Core>

class Model
{
public:

    Model();
    Model(double sampling_time);
    // ~Model(){};

    Eigen::MatrixXd SystemMatrix(Eigen::Vector3d x, Eigen::Vector2d u);
    Eigen::MatrixXd ControlMatrix(Eigen::Vector3d x, Eigen::Vector2d u);
    Eigen::VectorXd DynamicFunction(Eigen::Vector3d x, Eigen::Vector2d u);
    
    double sampling_time_;

    // Parameter:
    int nx;
    int nu;
    double length_base;

    // Kinematic/Dynamic constraints:
    double LIN_VEL_MAX;
    double LIN_VEL_MIN;
    double ANG_VEL_MAX;
    double ANG_VEL_MIN;

    // Dynamics:
    // Eigen::Vector3d dfdt_;

    // Function matrix:
    // Eigen::MatrixXd f;

    // System matrix
    // Eigen::MatrixXd A;

    // Control matrix
    // Eigen::MatrixXd B;

private:

    void init();

};