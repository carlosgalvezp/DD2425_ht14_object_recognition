#ifndef OBJECT_MODEL_3D_H
#define OBJECT_MODEL_3D_H
#include <vector>
#include <string>
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename Descriptor_T>
struct Object_Model
{
    std::string name_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_;
    typename pcl::PointCloud<Descriptor_T>::Ptr descriptors_;

    Object_Model(std::string name,
                 typename pcl::PointCloud<Descriptor_T>::Ptr descriptor)
        :name_(name), descriptors_(descriptor){}
};



#endif // OBJECT_MODEL_H
