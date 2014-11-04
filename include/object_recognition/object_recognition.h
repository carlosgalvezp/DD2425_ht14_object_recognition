#ifndef OBJECT_RECOGNITION_H
#define OBJECT_RECOGNITION_H

#include <iostream>
#include <ctime>
#include <ras_utils/ras_utils.h>
#include <object_recognition/feature_extractor.h>
#include <object_recognition/object_model.hpp>
#include <dirent.h>
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
#include <pcl/search/kdtree.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

const std::string models_rel_path_ = "/vision_data/"; // Path relative to /home/user

class Object_Recognition
{
public:
    Object_Recognition();
    std::string recognize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in);
private:
    Feature_Extractor feat_extractor;
    std::vector<Object_Model<pcl::PFHRGBSignature250> > objects_model_;
    std::vector<Object_Model<pcl::PointXYZRGB> > objects_keypoints_;

    int getCorrespondences(const pcl::PointCloud<pcl::PFHRGBSignature250>::ConstPtr &test_descriptors,
                           const pcl::PointCloud<pcl::PFHRGBSignature250>::ConstPtr &model_descriptors);
    void one_way_correspondences(const pcl::PointCloud<pcl::PFHRGBSignature250>::ConstPtr &source,
                                 const pcl::PointCloud<pcl::PFHRGBSignature250>::ConstPtr &target,
                                 std::vector<int>& correspondences);
    void load_models();
    void load_object_model(const std::string &path, const std::string &model_name);
};

#endif // OBJECT_RECOGNITION_H
