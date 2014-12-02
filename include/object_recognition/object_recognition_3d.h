#ifndef OBJECT_RECOGNITION_3D_H
#define OBJECT_RECOGNITION_3D_H

#include <iostream>
#include <object_recognition/object_extractor_3d.h>
#include <object_recognition/feature_extractor_3d.hpp>
#include <object_recognition/feature_matching_3d.hpp>
#include <ras_utils/types.h>

// ROS
#include "ros/ros.h"

#include <object_recognition/vfh_recognition.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh.h>

class Object_Recognition_3D
{
public:
    Object_Recognition_3D();
    void recognize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud, std::vector<double> &class_probabilities);
    void recognize_vfh(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                                          std::vector<double> &class_probabilities);
private:
    Object_Extractor_3D object_extractor;
    Feature_Extractor_3D<RAS_Utils::DescriptorExtractor, RAS_Utils::DescriptorType> feature_extractor_;
    Feature_Matching_3D<RAS_Utils::DescriptorExtractor, RAS_Utils::DescriptorType> feature_matching_;

    VFH_Recognition vfh_recognition_;
};





#endif // OBJECT_RECOGNITION_H
