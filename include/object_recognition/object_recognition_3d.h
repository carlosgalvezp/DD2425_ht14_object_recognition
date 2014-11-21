#ifndef OBJECT_RECOGNITION_3D_H
#define OBJECT_RECOGNITION_3D_H

#include <iostream>
#include <object_recognition/object_extractor_3d.h>
#include <object_recognition/feature_extractor_3d.hpp>
#include <object_recognition/feature_matching_3d.hpp>
#include <ras_utils/types.h>

// ROS
#include "ros/ros.h"

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh.h>

class Object_Recognition_3D
{
public:
    Object_Recognition_3D();
    int recognize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in, const pcl::PointXYZ &mass_center);

private:
    Object_Extractor_3D object_extractor;
    Feature_Extractor_3D<RAS_Utils::DescriptorExtractor, RAS_Utils::DescriptorType> feature_extractor;
    Feature_Matching_3D<RAS_Utils::DescriptorExtractor, RAS_Utils::DescriptorType> feature_matching;
};





#endif // OBJECT_RECOGNITION_H
