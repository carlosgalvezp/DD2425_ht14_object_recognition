#include <object_recognition/object_recognition_3d.h>

Object_Recognition_3D::Object_Recognition_3D()
{

}

int Object_Recognition_3D::recognize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in, const pcl::PointXYZ &mass_center)
{
    std::string result = "";
    // ** Segment the object, removing planes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    object_extractor.extract_object(cloud_in, mass_center, object_cloud);

    if(object_cloud->points.size() != 0)
    {
        // ** Extract descriptors
        pcl::PointCloud<RAS_Utils::DescriptorType>::Ptr descriptors(new pcl::PointCloud<RAS_Utils::DescriptorType>);
        feature_extractor.get_descriptors(object_cloud, descriptors);

        // ** Match with model database
        result = feature_matching.match(descriptors);
    }
    else
    {
        ROS_INFO("NO OBJECT FOUND");
    }
    // ** Output result
    if(result == "cube")
        return 0;
    else if (result == "ball")
        return 1;
    return -1;
}
