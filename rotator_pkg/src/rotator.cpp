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

void Rotator::Logger(int Roll_x, int Pitch_y, int Yaw_z, double Roll_x_radians,
                      double Pitch_y_radians, double Yaw_z_radians ,tf2::Quaternion myQuaternion)
{
    std::cout<<"[Roll_x] = "<<Roll_x<<std::endl;
    std::cout<<"[Pitch_y] = "<<Pitch_y<<std::endl;
    std::cout<<"[Yaw_z] = "<<Yaw_z<<std::endl;
    
    std::cout<<std::endl;

    std::cout<<"[Roll_x_radians] = "<<Roll_x_radians<<std::endl;
    std::cout<<"[Pitch_y_radians] = "<<Pitch_y_radians<<std::endl;
    std::cout<<"[Yaw_z_radians] = "<<Yaw_z_radians<<std::endl;

    std::cout<<std::endl;

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
   
    double Roll_x_radians = Roll_x * (M_PI/180);
    double Pitch_y_radians = Pitch_y * (M_PI/180);
    double Yaw_z_radians = Yaw_z * (M_PI/180);

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(Roll_x_radians, Pitch_y_radians, Yaw_z_radians);

    geometry_msgs::Point position;
    position.z = 0.5;

    geometry_msgs::Quaternion orientation;
    orientation.x = myQuaternion.getX();
    orientation.y = myQuaternion.getY();
    orientation.z = myQuaternion.getZ();
    orientation.w = myQuaternion.getW();

    Logger(Roll_x, Pitch_y, Yaw_z, Roll_x_radians, Pitch_y_radians, Yaw_z_radians, myQuaternion);

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