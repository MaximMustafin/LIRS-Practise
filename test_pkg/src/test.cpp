#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <iostream>

                                         // the best default:
const double NUM_DISPARITIES = 0;        // 0
const double BLOCK_SIZE = 21;            // 21
const double DISP12_MAX_DIFF = 1;        // 1
const double SPECKLE_RANGLE = 8;         // 8
const double SPECKLE_WINDOW_SIZE = 9;    // 9
const double UNIQUENESS_RATIO = 5;       // 5
const double TEXTURE_THRESHOLD = 507;    // 507
const double MIN_DISPARITY = -39;        // -39
const double PRE_FILTER_CAP = 61;        // 61
const double PRE_FILTER_SIZE = 5;        // 5
const std::string OPENCV_WINDOW_LEFT_CAMERA = "Left_camera";
const std::string OPENCV_WINDOW_RIGHT_CAMERA = "Right_camera";
const std::string OPENCV_WINDOW_OUTPUT_DEPTH_MAP = "Result_depth_map";

int main(int argc, char **argv)
{
    ROS_INFO_ONCE("Forming depth map...");
    std::string path_to_right_image = "/home/maxim/lirs_summer_practise_ws/src/test_pkg/src/right.png";
    std::string path_to_left_image = "/home/maxim/lirs_summer_practise_ws/src/test_pkg/src/left.png";

    cv::Mat right_image = cv::imread(path_to_right_image, cv::IMREAD_COLOR);
    cv::Mat left_image = cv::imread(path_to_left_image, cv::IMREAD_COLOR);

    if(left_image.empty())
        {
            std::cout<<"No frame to read"<<std::endl;       
        }
        else
        {
            if(right_image.empty())
            {
                std::cout<<"No frame to read"<<std::endl;
            }
            else
            {
                cv::Mat im_left = left_image.clone();
                cv::Mat im_right = right_image.clone();

                cv::Size image_size = im_left.size();
                cv::Mat disparity = cv::Mat(image_size.height, image_size.width, CV_8UC1);
                cv::Mat g1, g2, disp, disp8;
                cv::cvtColor(im_left, g1, cv::COLOR_BGR2GRAY);
                cv::cvtColor(im_right, g2, cv::COLOR_BGR2GRAY);

                cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(NUM_DISPARITIES, BLOCK_SIZE);

                sbm->setDisp12MaxDiff(DISP12_MAX_DIFF);
                sbm->setSpeckleRange(SPECKLE_RANGLE);
                sbm->setSpeckleWindowSize(SPECKLE_WINDOW_SIZE);
                sbm->setUniquenessRatio(UNIQUENESS_RATIO);
                sbm->setTextureThreshold(TEXTURE_THRESHOLD);
                sbm->setMinDisparity(MIN_DISPARITY);
                sbm->setPreFilterCap(PRE_FILTER_CAP);
                sbm->setPreFilterSize(PRE_FILTER_SIZE);
                sbm->compute(g1, g2, disparity);

                cv::normalize(disparity, disp8, 0, 255, cv::NORM_MINMAX, CV_8UC1);

                cv::namedWindow(OPENCV_WINDOW_LEFT_CAMERA, cv::WINDOW_FREERATIO);
                cv::imshow(OPENCV_WINDOW_LEFT_CAMERA, im_left);

                cv::namedWindow(OPENCV_WINDOW_RIGHT_CAMERA, cv::WINDOW_FREERATIO);
                cv::imshow(OPENCV_WINDOW_RIGHT_CAMERA, im_right);

                cv::namedWindow(OPENCV_WINDOW_OUTPUT_DEPTH_MAP, cv::WINDOW_FREERATIO);
                cv::imshow(OPENCV_WINDOW_OUTPUT_DEPTH_MAP, disp8);
                cv::waitKey(0);  

                return 0;
        }
    }
}