#include "rotator.h"

Rotator::Rotator(std::string modelname_)
{
    model_name = modelname_;
    client = node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

int getRandomNumber(int min, int max)
{
    static const double fraction = 1.0 / (static_cast<double>(RAND_MAX) + 1.0);
    return static_cast<int>(std::rand() * fraction * (max - min + 1) + min);
}

void Rotator::Logger(int Roll_x, int Pitch_y, int Yaw_z, tf2::Quaternion myQuaternion)
{
    std::cout<<"[Roll_x] = "<<Roll_x<<std::endl;
    std::cout<<"[Pitch_y] = "<<Pitch_y<<std::endl;
    std::cout<<"[Yaw_z] = "<<Yaw_z<<std::endl;

    std::cout<<"Quaternion:"<<std::endl;
    std::cout<<"    "<<"[x] = "<<myQuaternion.getX()<<std::endl;
    std::cout<<"    "<<"[y] = "<<myQuaternion.getY()<<std::endl;
    std::cout<<"    "<<"[z] = "<<myQuaternion.getZ()<<std::endl;
    std::cout<<"    "<<"[w] = "<<myQuaternion.getW()<<std::endl;
}

void Rotator::Start()
{
    int Roll_x = getRandomNumber(0, 360);
    int Pitch_y = getRandomNumber(0, 360);
    int Yaw_z = getRandomNumber(0, 360);
   
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(Roll_x, Pitch_y, Yaw_z);

    geometry_msgs::Point position;
    position.z = 0.5;

    geometry_msgs::Quaternion orientation;
    orientation.x = myQuaternion.getX();
    orientation.y = myQuaternion.getY();
    orientation.z = myQuaternion.getZ();
    orientation.w = myQuaternion.getW();

    Logger(Roll_x, Pitch_y, Yaw_z, myQuaternion);

    geometry_msgs::Pose pose;
    pose.position = position;
    pose.orientation = orientation;
    
    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = model_name;
    modelstate.pose = pose;

    gazebo_msgs::SetModelState srv;
    srv.request.model_state = modelstate;

    if(client.call(srv))
    {
        
        ROS_INFO("Model has been successfully randomly rotated");
    }
    else
    {
        ROS_ERROR("Failed to rotate! Error msg:%s", srv.response.status_message.c_str());
    }

}