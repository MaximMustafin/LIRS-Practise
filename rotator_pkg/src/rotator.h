#ifndef ROTATOR_H
#define ROTATOR_H

#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <locale>

class Rotator
{
    public:
        void Start(double);
        void Logger(int, int, int, double, double, double, tf2::Quaternion);
        Rotator(std::string);
        std::string model_name;
    
    private:
        ros::NodeHandle node;
        ros::ServiceClient client;
    
};

#endif // ROTATOR_H
