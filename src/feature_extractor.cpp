#include <object_recognition/feature_extractor.h>

Feature_Extractor::Feature_Extractor()
{
}

void Feature_Extractor::segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_out)
{

}

void Feature_Extractor::keypoint_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                                  pcl::PointCloud<pcl::PointXYZI>::ConstPtr &keypoints)
{

}

void Feature_Extractor::feature_extraction(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &keypoints)
{

}
