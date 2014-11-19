#include <iostream>
#include <string>
#include <sstream>
#include <dirent.h>
#include <termios.h>
#include <sstream>
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
class Vision_Training{

    typedef image_transport::ImageTransport ImageTransport;
    typedef image_transport::Publisher ImagePublisher;
    typedef image_transport::SubscriberFilter ImageSubFilter;

    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::Image,
                        sensor_msgs::Image> RGBD_Sync_Policy;

    typedef message_filters::Synchronizer<RGBD_Sync_Policy> RGBD_Sync;

public:
    Vision_Training(const ros::NodeHandle& n, std::string model_name, int frame_nr);
    void run();

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
    int frame_nr_;
    std::string model_name_;
};

// =============================================================================
// =============================================================================

int main(int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "vision_training2D");
    ros::NodeHandle n;

    // ** Create object recognition object
    Vision_Training o(n, argv[1], atoi(argv[2]));

    o.run();

    return 0;
}


Vision_Training::Vision_Training(const ros::NodeHandle& n, std::string model_name, int frame_nr)
    : n_(n), rgb_transport_(n), depth_transport_(n), frame_nr_(frame_nr), model_name_(model_name)
{
    // ** Subscribers

    //Char
    char_sub_ = n_.subscribe<std_msgs::Char>("/keyboard_listener/char", 5, &Vision_Training::Keyboard_Callback,this);

    //RGB and Depth images
    rgb_sub_.subscribe(rgb_transport_,
                       "/camera/rgb/image_raw", QUEUE_SIZE);
    depth_sub_.subscribe(depth_transport_,
                       "/camera/depth/image", QUEUE_SIZE);

    rgbd_sync_.reset(new RGBD_Sync(RGBD_Sync_Policy(QUEUE_SIZE), rgb_sub_, depth_sub_));
    rgbd_sync_->registerCallback(boost::bind(&Vision_Training::RGBD_Callback, this, _1, _2));

}

void Vision_Training::run()
{
    while(ros::ok())
    {
        if (c_ == 's')
        {
            std::stringstream ss;
            ss <<std::string(getenv("HOME")) << data_rel_path_ <<model_name_<<"/"<<model_name_<<"_"<<frame_nr_<<".jpg";

            std::cout << "Saving in "<<ss.str()<<std::endl;
            std::cout << cv::imwrite(ss.str(), cropped_img_)<<std::endl;
            ++frame_nr_;
            c_ = 0;
        }
        ros::spinOnce();
    }
}

void Vision_Training::RGBD_Callback(const sensor_msgs::ImageConstPtr &rgb_msg,
                                    const sensor_msgs::ImageConstPtr &depth_msg)
{
    // ** Convert ROS messages to OpenCV images
    cv_bridge::CvImageConstPtr rgb_ptr   = cv_bridge::toCvShare(rgb_msg);
    cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg);

    const cv::Mat& rgb_img   = rgb_ptr->image;
    const cv::Mat& depth_img = depth_ptr->image;

    // ** Store image
    cv::Mat rgb;
    cv::cvtColor(rgb_img, rgb, CV_BGR2RGB);

    // ** Show grid
    cv::Mat rgb_grid;
    rgb.copyTo(rgb_grid);

    cv::line(rgb_grid, cv::Point(0,CENTER_Y), cv::Point(IMG_COLS-1, CENTER_Y), cv::Scalar(0,0,255));
    cv::line(rgb_grid, cv::Point(CENTER_X,0), cv::Point(CENTER_X,IMG_ROWS-1), cv::Scalar(0,0,255));

    cv::line(rgb_grid, cv::Point(CENTER_X-SIZE/2.0,0), cv::Point(CENTER_X-SIZE/2.0,IMG_ROWS-1), cv::Scalar(0,0,255));
    cv::line(rgb_grid, cv::Point(CENTER_X+SIZE/2.0,0), cv::Point(CENTER_X+SIZE/2.0,IMG_ROWS-1), cv::Scalar(0,0,255));
    cv::line(rgb_grid, cv::Point(0,CENTER_Y-SIZE/2.0), cv::Point(IMG_COLS-1, CENTER_Y-SIZE/2.0), cv::Scalar(0,0,255));
    cv::line(rgb_grid, cv::Point(0,CENTER_Y+SIZE/2.0), cv::Point(IMG_COLS-1, CENTER_Y+SIZE/2.0), cv::Scalar(0,0,255));


    cv::imshow("Grid", rgb_grid);
    cv::waitKey(1);

    // ** Crop image
    cv::Rect roi(CENTER_X-SIZE/2.0,CENTER_Y-SIZE/2.0, SIZE, SIZE);
    cropped_img_ = rgb(roi);
    cv::imshow("Cropped", cropped_img_);
    cv::waitKey(1);
}

void Vision_Training::Keyboard_Callback(const std_msgs::CharConstPtr &char_msg)
{
    c_ = char_msg->data;
}
