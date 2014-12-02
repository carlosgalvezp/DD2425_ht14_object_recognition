#include <object_recognition/object_recognition_3d.h>

Object_Recognition_3D::Object_Recognition_3D()
{

}

void Object_Recognition_3D::recognize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                                      std::vector<double> &class_probabilities)
{
    if(object_cloud != 0 && object_cloud->size() != 0)
    {
        // ** Extract descriptors
        pcl::PointCloud<RAS_Utils::DescriptorType>::Ptr descriptors(new pcl::PointCloud<RAS_Utils::DescriptorType>);
        feature_extractor_.get_descriptors(object_cloud, descriptors);

        // ** Match with model database
        feature_matching_.match(descriptors, class_probabilities);
    }
    else
    {
        ROS_ERROR("[Object_Recognition_3D::recognize] Object_cloud = 0");
    }
}


void Object_Recognition_3D::recognize_vfh(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                                      std::vector<double> &class_probabilities)
{
    if(object_cloud != 0 && object_cloud->size() != 0)
    {
        // ** Extract descriptors
        pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);
        vfh_recognition_.computeDescriptor(object_cloud, descriptors);

        // ** Match with model database
        feature_matching_.match_vfh(descriptors, class_probabilities);
    }
    else
    {
        ROS_ERROR("[Object_Recognition_3D::recognize] Object_cloud = 0");
    }
}
