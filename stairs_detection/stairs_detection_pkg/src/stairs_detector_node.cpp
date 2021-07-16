#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stairs_detection_pkg/stairs_detector.h>
#include <dynamic_reconfigure/server.h>
#include "stairs_detection_pkg/Stairs_DetectorConfig.h"
#include <boost/bind.hpp>

Stairs_Detector stairs_detector;
std::vector<cv::Point> tmp_bounding_box;
std::vector<cv::Point> show_bounding_box;
Stairs_Detector_Params param;
bool stair_detected = false;
int detected_count = 0;
int max_valid_detection = 5;

void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_ONCE("Recevied RGB Image");
    try
    {
        cv::Mat rgb_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        if (stair_detected) {
            stairs_detector.drawBox(rgb_image, show_bounding_box);
        }
        cv::imshow("Result", rgb_image);
        cv::waitKey(5);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to '8UC3'.", msg->encoding.c_str());
    }
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_ONCE("Recevied depth image");
    try
    {
        cv::Mat depth_image = cv_bridge::toCvCopy(msg, "8UC1")->image;
        if (stairs_detector.getStairs(depth_image, tmp_bounding_box)) {
            detected_count++;
            ROS_INFO("Found potential stairs");
            if (detected_count >= max_valid_detection) {
                stair_detected = true;
                show_bounding_box = tmp_bounding_box;
                ROS_INFO("Found stairs");
            }
        } else {
            stair_detected = false;
            detected_count = 0;
            show_bounding_box.clear();
            ROS_INFO("Can't find stairs");
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to '8UC1'.", msg->encoding.c_str());
    }
}

void reconfigureCB(stairs_detection_pkg::Stairs_DetectorConfig &config, uint32_t level)
{
    param.show_result = config.show_result;
    param.debug = config.debug;
    param.ignore_invalid = config.ignore_invalid;
    param.fill_invalid = config.fill_invalid;
    param.use_laplacian = config.use_laplacian;

    // canny
    param.canny_low_threshold = config.canny_low_threshold;
    param.canny_ratio = config.canny_ratio;
    param.canny_kernel_size = config.canny_kernel_size;

    // hough transfrom
    param.hough_min_line_length = config.hough_min_line_length;
    param.hough_max_line_gap = config.hough_max_line_gap;
    param.hough_threshold = config.hough_threshold;
    param.hough_rho = config.hough_rho;
    param.hough_theta = config.hough_theta;

    // filter by slope histogram
    param.filter_slope_hist_bin_width = config.filter_slope_hist_bin_width;

    // filter by hard slope
    param.filter_slope_bandwidth = config.filter_slope_bandwidth;

    // merge parameters
    param.merge_max_dist_diff = config.merge_max_dist_diff;
    param.merge_max_angle_diff = config.merge_max_angle_diff;
    param.merge_close_count = config.merge_close_count;

    // bounding box
    param.minimum_line_num = config.minimum_line_num;
    max_valid_detection = config.max_valid_detection;

    stairs_detector.setParam(param);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stairs_detector");
    ros::NodeHandle nh;
    cv::namedWindow("Result");
    cv::startWindowThread();

    param.fill_invalid = false;
    param.ignore_invalid = false;
    param.use_laplacian = false;
    stairs_detector.setParam(param);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_depth = it.subscribe("depth/image", 1, depthImageCallback);
    image_transport::Subscriber sub_rgb = it.subscribe("rgb/image", 1, rgbImageCallback);

    dynamic_reconfigure::Server<stairs_detection_pkg::Stairs_DetectorConfig> dsrv(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<stairs_detection_pkg::Stairs_DetectorConfig>::CallbackType cb = boost::bind(&reconfigureCB, _1, _2);
    dsrv.setCallback(cb);

    ROS_INFO_ONCE("Stairs detector waiting for images");
    ros::spin();
    cv::destroyWindow("Result");
    return 0;
}