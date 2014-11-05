#include <iostream>
#include <ctime>
#include <ras_utils/ras_utils.h>
#include <object_recognition/object_recognition.h>
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

class Object_Recognition_Node{

    typedef image_transport::ImageTransport ImageTransport;
    typedef image_transport::Publisher ImagePublisher;
    typedef image_transport::SubscriberFilter ImageSubFilter;

    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::Image,
                        sensor_msgs::Image> RGBD_Sync_Policy;

    typedef message_filters::Synchronizer<RGBD_Sync_Policy> RGBD_Sync;

public:
    Object_Recognition_Node(const ros::NodeHandle& n);

private:
    ros::NodeHandle n_;

    image_transport::ImageTransport rgb_transport_;
    image_transport::ImageTransport depth_transport_;

    image_transport::SubscriberFilter rgb_sub_;
    image_transport::SubscriberFilter depth_sub_;

    ros::Publisher speaker_pub_;

    boost::shared_ptr<RGBD_Sync> rgbd_sync_;

    ros::Subscriber pcl_sub_;

    bool process_PCL_;

    Object_Recognition obj_recognition;
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
    // ** Init node
    ros::init(argc, argv, "object_recognition");
    ros::NodeHandle n;

    // ** Create object recognition object
    Object_Recognition_Node o(n);
    return 0;
}


Object_Recognition_Node::Object_Recognition_Node(const ros::NodeHandle& n)
    : n_(n), rgb_transport_(n), depth_transport_(n), process_PCL_(false)
{
    // ** Publishers
    speaker_pub_ = n_.advertise<std_msgs::String>("object_recognition/speaker", 1000);

    // ** Subscribers
//    rgb_sub_.subscribe(rgb_transport_,
//                       "/camera/rgb/image_raw", QUEUE_SIZE);
//    depth_sub_.subscribe(depth_transport_,
//                       "/camera/depth/image", QUEUE_SIZE);

//    rgbd_sync_.reset(new RGBD_Sync(RGBD_Sync_Policy(QUEUE_SIZE), rgb_sub_, depth_sub_));
//    rgbd_sync_->registerCallback(boost::bind(&Object_Recognition::RGBD_Callback, this, _1, _2));

    pcl_sub_ = n_.subscribe("/camera/depth_registered/points", QUEUE_SIZE, &Object_Recognition_Node::PCL_Callback, this);
}

void Object_Recognition_Node::RGBD_Callback(const sensor_msgs::ImageConstPtr &rgb_msg,
                                       const sensor_msgs::ImageConstPtr &depth_msg)
{
    // ** Convert ROS messages to OpenCV images
    cv_bridge::CvImageConstPtr rgb_ptr   = cv_bridge::toCvShare(rgb_msg);
    cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg);

    const cv::Mat& rgb_img   = rgb_ptr->image;
    const cv::Mat& depth_img = depth_ptr->image;
    cv::imshow("RGB", rgb_img);
    cv::imshow("DEPTH", depth_img);
    cv::waitKey(1);

    // ** Process image
}

void Object_Recognition_Node::PCL_Callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_msg)
{
    std::string object = obj_recognition.recognize(pcl_msg);
    // ** Publish
}
