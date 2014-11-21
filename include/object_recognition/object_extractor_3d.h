#ifndef OBJECT_EXTRACTOR_3D_H
#define OBJECT_EXTRACTOR_3D_H

#include <ras_utils/pcl_utils.h>

// ROS
#include "ros/ros.h"

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/LU>

#define CLOUD_RES 0.001
class Object_Extractor_3D
{
public:
    Object_Extractor_3D();

    void extract_object(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                        const pcl::PointXYZ &mass_center,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr      &cloud_out);
private:
    void removePlanes(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr      &cloud_out);

    void preprocess(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                    const pcl::PointXYZ &mass_center,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr      &cloud_out);
    void extractCluster(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr      &cloud_out);

    void extractCluster2(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                         const pcl::PointXYZ &mass_center,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr      &cloud_out);
};

#endif // OBJECT_EXTRACTOR_H
