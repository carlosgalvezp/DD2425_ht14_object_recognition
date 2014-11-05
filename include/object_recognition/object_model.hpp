#ifndef OBJECT_MODEL_H
#define OBJECT_MODEL_H
#include <vector>
#include <string>
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

template <typename Descriptor_T>
struct Object_Model
{
    std::string name_;
    typename pcl::PointCloud<Descriptor_T>::Ptr descriptors_;

    Object_Model(std::string name, typename pcl::PointCloud<Descriptor_T>::Ptr descriptor)
        :name_(name), descriptors_(descriptor){}
};



#endif // OBJECT_MODEL_H
