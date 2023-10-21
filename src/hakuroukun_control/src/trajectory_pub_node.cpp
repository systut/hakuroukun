#include <eigen3/Eigen/Core>
#include <cmath>
#include <mutex>
#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"

class TrajectoryPubNode
{
public:

TrajectoryPubNode(ros::NodeHandle nh, ros::NodeHandle private_nh){

    nh_ = nh;
    private_nh_ = private_nh;
    std::string file_path = "src/data/uturn.csv";
    // std::string file_path = "src/data/lemniscate_of_gerono_ref.csv";
    ReadReference(file_path);
}

void PublishTrajectory(){

    ros::Publisher traj_pub_;
    traj_pub_ = nh_.advertise<sdv_msgs::Trajectory>("/trajectory", 1);

    while (ros::ok())
    {
        traj_pub_.publish(trajectory_msg_);
    }
}

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    sdv_msgs::Trajectory trajectory_msg_;

std::tuple<int, int> GetRowAndColumn(std::string file_path)
{
    std::ifstream file(file_path);
    std::string line;
    int row_num = 0;
    int col_num = 0;
    
    while (std::getline(file, line)) {
        row_num++;
        std::istringstream lineStream(line);
        std::string cell;
        col_num = 0;
        while (std::getline(lineStream, cell, ',')) {
            col_num++;
        }
    }
    // Close the CSV file
    file.close();

    return std::make_tuple(row_num, col_num);
}

void ReadReference(std::string file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open the CSV file." << std::endl;
    }

    int row_num, col_num;
    std::tie(row_num, col_num) = GetRowAndColumn(file_path);

    // messages resize
    trajectory_msg_.points.resize(row_num-1);

    // Read the CSV file line by line
    std::string data_row;
    int currentRow = 0;

    while (std::getline(file, data_row)) {

        std::vector<float> data_array;
        std::istringstream data_row_stream(data_row);
        std::string data_cell;
        char delimiter = ',';

        while (std::getline(data_row_stream, data_cell, delimiter)) {

            if (currentRow > 0)
            {
                float value;
                value = std::stof(data_cell); // Convert the token to a double
                data_array.push_back(value);
            } 
        }

        if (data_array.size() > 0) {
            trajectory_msg_.points[currentRow-1].x = data_array[1];
            trajectory_msg_.points[currentRow-1].y = data_array[2];
            trajectory_msg_.points[currentRow-1].heading = data_array[3];
            trajectory_msg_.points[currentRow-1].velocity_mps = data_array[4];
            trajectory_msg_.points[currentRow-1].steering_angle = data_array[5]; 

        }        
        currentRow++;
    }
    // Close the CSV file
    file.close(); 
}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_pub_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    TrajectoryPubNode trajectory_pub_node(nh, private_nh);
    trajectory_pub_node.PublishTrajectory();

    return 0;
}