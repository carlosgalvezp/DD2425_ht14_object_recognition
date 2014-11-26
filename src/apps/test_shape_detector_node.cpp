#include <iostream>
#include <string>
#include <sstream>
#include <dirent.h>
#include <termios.h>
#include <sstream>
#include <object_recognition/shape_detector_2d.h>
// ROS
#include "ros/ros.h"
#include <std_msgs/Char.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define QUEUE_SIZE 1

#define IMG_ROWS 480
#define IMG_COLS 640
#define CENTER_X IMG_COLS/2
#define CENTER_Y 350
#define SIZE     50

const std::string data_rel_path_ = "/color_data/";

int getch();
class Test_Shape_Detector_Node{

    typedef image_transport::ImageTransport ImageTransport;
    typedef image_transport::Publisher ImagePublisher;
    typedef image_transport::SubscriberFilter ImageSubFilter;

    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::Image,
                        sensor_msgs::Image> RGBD_Sync_Policy;

    typedef message_filters::Synchronizer<RGBD_Sync_Policy> RGBD_Sync;

public:
    Test_Shape_Detector_Node(const ros::NodeHandle& n);

private:
    ros::NodeHandle n_;

    image_transport::ImageTransport rgb_transport_;
    image_transport::ImageTransport depth_transport_;

    image_transport::SubscriberFilter rgb_sub_;
    image_transport::SubscriberFilter depth_sub_;

    boost::shared_ptr<RGBD_Sync> rgbd_sync_;

    ros::Subscriber pcl_sub_;

    cv::Mat rgb_img_, depth_img_;

    ros::Subscriber char_sub_;
    /**
     * @brief Callback to process RGB and Depth image
     * @param rgb_msg
     * @param depth_msg
     */
    void RGBD_Callback(const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg);
    cv::Mat cropped_img_;

    void Keyboard_Callback(const std_msgs::CharConstPtr& char_msg);

    char c_;
    int frame_nr_, circle_nr_;
    std::string model_name_;
    Shape_Detector_2D shape_detector;
};

// =============================================================================
// =============================================================================

int main(int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "Test_Shape_Detector_Camera");
    ros::NodeHandle n;

    // ** Create object recognition object
    Test_Shape_Detector_Node o(n);

    ros::spin();
    return 0;
}


Test_Shape_Detector_Node::Test_Shape_Detector_Node(const ros::NodeHandle& n)
    : n_(n), rgb_transport_(n), depth_transport_(n), frame_nr_(0), circle_nr_(0)
{
    // ** Subscribers

    //Char
    char_sub_ = n_.subscribe<std_msgs::Char>("/keyboard_listener/char", 5, &Test_Shape_Detector_Node::Keyboard_Callback,this);

    //RGB and Depth images
    rgb_sub_.subscribe(rgb_transport_,
                       TOPIC_CAMERA_RGB, QUEUE_SIZE);
    depth_sub_.subscribe(depth_transport_,
                       TOPIC_CAMERA_DEPTH, QUEUE_SIZE);

    rgbd_sync_.reset(new RGBD_Sync(RGBD_Sync_Policy(QUEUE_SIZE), rgb_sub_, depth_sub_));
    rgbd_sync_->registerCallback(boost::bind(&Test_Shape_Detector_Node::RGBD_Callback, this, _1, _2));

}


void Test_Shape_Detector_Node::RGBD_Callback(const sensor_msgs::ImageConstPtr &rgb_msg,
                                    const sensor_msgs::ImageConstPtr &depth_msg)
{

    // ** Convert ROS messages to OpenCV images
    cv_bridge::CvImageConstPtr rgb_ptr   = cv_bridge::toCvShare(rgb_msg);
    cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg);

    const cv::Mat& rgb_img   = rgb_ptr->image;
    const cv::Mat& depth_img = depth_ptr->image;
    cv::Mat mask = cv::Mat::zeros(rgb_img.rows, rgb_img.cols, CV_8UC1);
    for(unsigned int u = 250; u < 375; ++u)
    {
        for(unsigned int v=200; v < 400; ++v)
        {
            mask.at<uint8_t>(v,u) = 255;
        }
    }
    cv::Mat imgcropped;
    rgb_img.copyTo(imgcropped);
    ++frame_nr_;
    if(frame_nr_ > 50)
    {

//        if(shape_detector.circle_detection(imgcropped, true))
//            ++circle_nr_;
//        ROS_INFO("Circle detection: %.1f %%",100.0*(double)circle_nr_/(frame_nr_-50));
        shape_detector.vertical_lines(imgcropped, true);
    }

}

void Test_Shape_Detector_Node::Keyboard_Callback(const std_msgs::CharConstPtr &char_msg)
{
    c_ = char_msg->data;
}

