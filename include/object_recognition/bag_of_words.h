#ifndef BAGOFWORDS_H
#define BAGOFWORDS_H

#include <string>
#include <object_recognition/feature_extractor.h>
// ROS
//#include "ros/ros.h"
//#include <std_msgs/String.h>
//#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/PointCloud2.h>

//#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>
//#include <cv_bridge/cv_bridge.h>

// PCL
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// OpenCV
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
static const std::string train_directory_ = "~/ras_train_data/pcd/";

class Bag_of_Words
{
public:
    Bag_of_Words();

    /**
     * @brief Loads a set of labeled point clouds and builds the model
     */
    void train();

    /**
     * @brief Given the current point cloud, estimates the type of object
     * @return
     */
    std::string estimate_class(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in) const;

private:
    Feature_Extractor feat_extractor_;
};

#endif // BAGOFWORDS_H
