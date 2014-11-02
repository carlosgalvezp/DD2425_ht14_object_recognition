#include <iostream>
#include <string>
#include <sstream>
// ROS
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define QUEUE_SIZE 1000
#define PUBLISH_RATE 1.0/5.0  //Save data every 5 seconds
class Vision_Training{

    typedef image_transport::ImageTransport ImageTransport;
    typedef image_transport::Publisher ImagePublisher;
    typedef image_transport::SubscriberFilter ImageSubFilter;

    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::Image,
                        sensor_msgs::Image> RGBD_Sync_Policy;

    typedef message_filters::Synchronizer<RGBD_Sync_Policy> RGBD_Sync;

public:
    Vision_Training(const ros::NodeHandle& n, std::string path, std::string model_name);
    /**
     * @brief Save data at a given frequency
     */
    void run();
private:
    ros::NodeHandle n_;
    std::string model_name_, path_;
    int n_item_;

    image_transport::ImageTransport rgb_transport_;
    image_transport::ImageTransport depth_transport_;

    image_transport::SubscriberFilter rgb_sub_;
    image_transport::SubscriberFilter depth_sub_;

    boost::shared_ptr<RGBD_Sync> rgbd_sync_;

    ros::Subscriber pcl_sub_;

    cv::Mat rgb_img_, depth_img_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    /**
     * @brief Callback to process RGB and Depth image
     * @param rgb_msg
     * @param depth_msg
     */
    void RGBD_Callback(const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg);

    /**
     * @brief Callback to process registered point cloud
     * @param pcl_msg
     */
    void PCL_Callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_msg);

};

// =============================================================================
// =============================================================================

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        ROS_ERROR("Usage: rosrun vision_training <path> <model_name>");
        return -1;
    }

    // ** Init node
    ROS_INFO("Saving data for model of %s",argv[1]);
    ros::init(argc, argv, "vision_training");
    ros::NodeHandle n;

    // ** Create object recognition object
    Vision_Training o(n, argv[1], argv[2]);
    o.run();

    return 0;
}


Vision_Training::Vision_Training(const ros::NodeHandle& n, std::string path, std::string model_name)
    : n_(n), model_name_(model_name), path_(path), n_item_(1), rgb_transport_(n), depth_transport_(n)
{
    // ** Subscribers
    // Point Cloud
    pcl_sub_ = n_.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >
            ("/camera/depth_registered/points", QUEUE_SIZE, &Vision_Training::PCL_Callback,this);

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
    // ** Publish data
    ros::Rate rate(PUBLISH_RATE);

    while(ros::ok())
    {
        ROS_INFO("Reading data...");
        ros::spinOnce(); //Get data

        // ** Save pcd and images
        ROS_INFO("Saving data %u...", n_item_);
        // PCD
        if (cloud_ != 0)
        {
            std::stringstream ss, ss2, ss3;
            ss << path_<<model_name_<<"_"<<n_item_<<".pcd";
            pcl::io::savePCDFile(ss.str(), *cloud_);

            // Images
            ss2 << path_ <<model_name_<<"_RGB_"<<n_item_<<".png";
            ss3 << path_ << model_name_<<"_D_"<<n_item_<<".png";
            cv::imwrite(ss2.str(), rgb_img_);
            cv::imwrite(ss3.str(), depth_img_);

            ++n_item_;
        }

        // ** Sleep
        rate.sleep();
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
    rgb_img_ = rgb_img;
    depth_img_ = depth_img;
}

void Vision_Training::PCL_Callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_msg)
{
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*pcl_msg, *cloud_);
}
