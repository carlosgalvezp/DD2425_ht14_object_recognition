#include <iostream>
#include <ctime>
#include <ras_utils/ras_utils.h>
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

// Services
#include <ras_srv_msgs/Recognition.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define QUEUE_SIZE 10

void nullDeleter(void*);

class Object_Recognition_2D_Node{

    typedef image_transport::ImageTransport ImageTransport;
    typedef image_transport::Publisher ImagePublisher;
    typedef image_transport::SubscriberFilter ImageSubFilter;

    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::Image,
                        sensor_msgs::Image> RGBD_Sync_Policy;

    typedef message_filters::Synchronizer<RGBD_Sync_Policy> RGBD_Sync;

public:
    Object_Recognition_2D_Node(const ros::NodeHandle& n);

private:
    ros::NodeHandle n_;

    image_transport::ImageTransport rgb_transport_;
    image_transport::ImageTransport depth_transport_;

    image_transport::SubscriberFilter rgb_sub_;
    image_transport::SubscriberFilter depth_sub_;

    ros::Publisher speaker_pub_;

    boost::shared_ptr<RGBD_Sync> rgbd_sync_;

    ros::Subscriber pcl_sub_;

    ros::ServiceServer service_;
    bool process_PCL_;

    bool Recognition_Callback(ras_srv_msgs::Recognition::Request  &req,
                              ras_srv_msgs::Recognition::Response &res);
    std::string recognize(const cv::Mat &rgb_img, const cv::Mat &mask);

};
// =============================================================================
// =============================================================================

int main(int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "object_recognition");
    ros::NodeHandle n;

    // ** Create object recognition object
    Object_Recognition_2D_Node o(n);

    ros::spin();
    return 0;
}

Object_Recognition_2D_Node::Object_Recognition_2D_Node(const ros::NodeHandle& n)
    : n_(n), rgb_transport_(n), depth_transport_(n), process_PCL_(false)
{
    // ** Publishers
    speaker_pub_ = n_.advertise<std_msgs::String>("/espeak/string", 1000);

    // ** Services
    service_ = n_.advertiseService("/object_recognition/recognition",
                                   &Object_Recognition_2D_Node::Recognition_Callback,this);
}

void nullDeleter(void*) {}

bool Object_Recognition_2D_Node::Recognition_Callback(ras_srv_msgs::Recognition::Request  &req,
                                                      ras_srv_msgs::Recognition::Response &res)
{
    // ** Convert ROS messages to OpenCV images and scale
    boost::shared_ptr<sensor_msgs::Image> req_rgb_ptr(&req.rgb_img, nullDeleter);
    boost::shared_ptr<sensor_msgs::Image> req_mask_ptr(&req.mask, nullDeleter);

    cv_bridge::CvImageConstPtr rgb_ptr     = cv_bridge::toCvShare(req_rgb_ptr);
    cv_bridge::CvImageConstPtr mask_ptr    = cv_bridge::toCvShare(req_mask_ptr);

    const cv::Mat& rgb_img     = rgb_ptr->image;
    const cv::Mat& mask        = mask_ptr->image;

    // ** 2D object recognition
    std::string object = recognize(rgb_img, mask);

    ros::Rate r(1.0/5.0);
    r.sleep();
    // ** Publish to speaker node
    std::string str = ("I see " + object);
    std_msgs::String msg;
    msg.data = str;
    speaker_pub_.publish(msg);

    res.result = 1;
    return true;
}

std::string Object_Recognition_2D_Node::recognize(const cv::Mat &rgb_img, const cv::Mat &mask)
{
    std::string result = "a red cube";
    return result;
}
