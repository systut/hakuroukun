#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <cmath>
#include <mutex>
#include <boost/make_shared.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include "../../include/model.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double sampling_time = 0.05;
    Model model(sampling_time);

    Eigen::Vector3d x_ref;
    Eigen::Vector2d u_ref;
    Eigen::MatrixXd A_d(3,3);
    Eigen::MatrixXd B_d(3,2);
    Eigen::Vector3d f_d;

    x_ref << 2,2,0;
    u_ref << 1,0;

    std::cout << x_ref << std::endl;
    std::cout << u_ref << std::endl;

    A_d = model.SystemMatrix(x_ref, u_ref);
    B_d = model.ControlMatrix(x_ref, u_ref);
    f_d = model.DynamicFunction(x_ref, u_ref);

    std::cout << "System matrix" << std::endl;
    std::cout << A_d << std::endl;
    std::cout << "Control matrix" << std::endl;
    std::cout << B_d << std::endl;
    std::cout << "Dynamic matrix" << std::endl;
    std::cout << f_d << std::endl;

    return 0;
}