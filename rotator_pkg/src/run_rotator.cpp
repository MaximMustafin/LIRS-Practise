#include "rotator.h"
#include <boost/algorithm/string.hpp>

int main(int argc, char **argv)
{
    std::string doContinue = "Y";
    std::cout<<"Enter your model's name which you want to rotate"<<std::endl;
    std::cout<<"Model's name = ";
    std::string model_name;
    std::cin>>model_name;

    while(doContinue == "Y")
    {
        ros::init(argc, argv, "rotator");

        Rotator rotator = Rotator(model_name);
        rotator.Start();

        std::cout<<"Do rotate one more time? [Y][N]";
        std::cin>>doContinue;
    }
    
}