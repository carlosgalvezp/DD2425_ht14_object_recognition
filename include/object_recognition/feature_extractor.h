#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <vector>
#include <iostream>
#include <ras_utils/ras_utils.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#define CLOUD_RES  0.005 // [m]
#define KEYPOINT_RES 0.01 // [m]

/**
 * @brief Takes as an input a point cloud image containing one object, and
 * performs the following operations:
 * 1.- Segmention of the object
 * 2.- Detection of interest points
 * 3.- Feature extraction
 */
class Feature_Extractor
{
public:
    /**
     * @brief Default constructor
     */
    Feature_Extractor();

    /**
     * @brief Given a raw input point cloud extracts the object's feature descriptors
     * @param cloud_in the raw input point cloud
     */
    void get_descriptors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                               pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &descriptors);

private:
    /**
     * @brief Preprocesses the input cloud: pass-through filter, sampling, region of interest
     * @param cloud_in  input cloud
     * @param cloud_out output cloud
     */
    void preprocessing(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

    /**
     * @brief Segments the object from the scene
     * @param cloud_in  input cloud, containing the whole scene
     * @param cloud_out output cloud, containing only the 3D object
     */
    void segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
    /**
     * @brief Extracts keypoints from the object
     * @param cloud_in input cloud of the object
     * @param keypoints cloud of keypoints
     */
    void keypoint_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints);

    /**
     * @brief Extracts feature descriptors for each keypoint
     * @param keypoints the input keypoints
     */
    void descriptor_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints,
                               const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                               pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &descriptors);
};

#endif // FEATURE_EXTRACTOR_H
