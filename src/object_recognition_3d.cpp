#include <object_recognition/object_recognition_3d.h>

Object_Recognition_3D::Object_Recognition_3D()
{

}

int Object_Recognition_3D::recognize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in)
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
    if(result == "cube")
        return 0;
    else if (result == "sphere")
        return 1;
    return -1;
}
