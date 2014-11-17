#include <object_recognition/object_recognition_3d.h>

Object_Recognition_3D::Object_Recognition_3D()
{

}

std::string Object_Recognition_3D::recognize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in)
{
    // ** Segment the object, removing planes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    object_extractor.extract_object(cloud_in, object_cloud);

    // ** Extract descriptors
    pcl::PointCloud<RAS_Utils::DescriptorType>::Ptr descriptors(new pcl::PointCloud<RAS_Utils::DescriptorType>);
    feature_extractor.get_descriptors(object_cloud, descriptors);

    // ** Match with model database
    std::string result = feature_matching.match(descriptors);

    // ** Output result
    return result;
}
