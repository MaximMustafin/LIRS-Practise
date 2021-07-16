#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

class DepthConverter
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub_left_camera;
    image_transport::Subscriber sub_right_camera;
    image_transport::Publisher pub_depth_map;

    bool left_camera_msg_updated = false;
    bool right_camera_msg_updated = false;
    cv::Mat previous_left_camera_image;
    cv::Mat previous_right_camera_image;
    std_msgs::Header previous_camera_msg_header;

    private:                                 // the best default:
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

    public:
        DepthConverter():it(nh)
        {
            sub_left_camera = it.subscribe("left_camera", 1, &DepthConverter::leftCameraCallback, this);
            sub_right_camera = it.subscribe("right_camera", 1, &DepthConverter::rightCameraCallback, this);
            pub_depth_map = it.advertise("/depth_map", 1000);

            // cv::namedWindow(OPENCV_WINDOW_LEFT_CAMERA, cv::WINDOW_FREERATIO);
            // cv::namedWindow(OPENCV_WINDOW_RIGHT_CAMERA, cv::WINDOW_FREERATIO);
            cv::namedWindow(OPENCV_WINDOW_OUTPUT_DEPTH_MAP, cv::WINDOW_FREERATIO);
        }

        ~DepthConverter()
        {
            // cv::destroyWindow(OPENCV_WINDOW_LEFT_CAMERA);
            // cv::destroyWindow(OPENCV_WINDOW_RIGHT_CAMERA);
            cv::destroyWindow(OPENCV_WINDOW_OUTPUT_DEPTH_MAP);
        }

    void getDepthMap()
    {
        ROS_INFO_ONCE("Forming depth map...");

        cv::Mat left_image = previous_left_camera_image.clone();
        cv::Mat right_image = previous_right_camera_image.clone();
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

                // cv::imshow(OPENCV_WINDOW_LEFT_CAMERA, im_left);
                // cv::imshow(OPENCV_WINDOW_RIGHT_CAMERA, im_right);
                cv::imshow(OPENCV_WINDOW_OUTPUT_DEPTH_MAP, disp8);

                cv::waitKey(1);

                cv_bridge::CvImage out_depth_map;
                out_depth_map.header = previous_camera_msg_header;
                out_depth_map.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
                out_depth_map.image = disp8;
                
                pub_depth_map.publish(out_depth_map.toImageMsg());
                ROS_INFO_ONCE("Depth map is being published to /depth_map topic...");               
            }
        }   
    }

    private:
    void leftCameraCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO_ONCE("Recevied image from left camera");

        try
        {      
            previous_left_camera_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("leftCameraCallback: Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }

        if(!previous_left_camera_image.empty())
        {
            left_camera_msg_updated = true;
        }

        if(left_camera_msg_updated && right_camera_msg_updated)
        {
            previous_camera_msg_header = msg->header;
            getDepthMap();
            previous_left_camera_image.release();
            previous_right_camera_image.release();
            left_camera_msg_updated = false;
            right_camera_msg_updated = false;
        }
    }

    void rightCameraCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO_ONCE("Recevied image from right camera");

        try
        {      
            previous_right_camera_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("rightCameraCallback: Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }

        if(!previous_right_camera_image.empty())
        {
            right_camera_msg_updated = true;
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_converter");
    DepthConverter depth_converter;
    ROS_INFO_ONCE("Waiting for images");
    ros::spin();
    ROS_INFO_ONCE("Stopping publishing depth map");
    return 0;
}