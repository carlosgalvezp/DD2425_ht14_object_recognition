#ifndef FEATURE_EXTRACTOR_3D_H
#define FEATURE_EXTRACTOR_3D_H

#include <vector>
#include <iostream>
#include <ras_utils/ras_utils.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#define CLOUD_RES  0.001 // [m]
#define KEYPOINT_RES 0.005 // [m]
/**
 * @brief Takes as an input a point cloud image containing one object, and
 * performs the following operations:
 * 1.- Segmention of the object
 * 2.- Detection of interest points
 * 3.- Feature extraction
 */
template<typename DescriptorExtractor, typename DescriptorType>
class Feature_Extractor_3D
{
public:
    /**
     * @brief Default constructor
     */
    Feature_Extractor_3D();

    /**
     * @brief Given a raw input point cloud extracts the object's feature descriptors
     * @param cloud_in the raw input point cloud
     */
    void get_descriptors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                      typename pcl::PointCloud<DescriptorType>::Ptr &descriptors);

    void get_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                       typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints);
private:    
    /**
     * @brief Extracts keypoints from the object
     * @param cloud_in input cloud of the object
     * @param keypoints cloud of keypoints
     */
    void keypoint_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints);

    /**
     * @brief Extracts feature descriptors for each keypoint
     * @param keypoints the input keypoints
     */
    void descriptor_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints,
                               const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                            typename pcl::PointCloud<DescriptorType>::Ptr &descriptors);
};

// ==========================================================================================
// ==========================================================================================
template<typename DescriptorExtractor, typename DescriptorType>
Feature_Extractor_3D<DescriptorExtractor, DescriptorType>::Feature_Extractor_3D()
{
}

template<typename DescriptorExtractor, typename DescriptorType>
void Feature_Extractor_3D<DescriptorExtractor, DescriptorType>::get_descriptors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                                                                          typename pcl::PointCloud<DescriptorType>::Ptr &descriptors)
{
    // ** Downsample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (object_cloud);
    vg.setLeafSize (CLOUD_RES, CLOUD_RES, CLOUD_RES);
    vg.filter (*object_downsampled);

    // ** Extract keypoints
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);
    keypoint_extraction(object_downsampled, keypoints);

    // ** Get descriptor
    descriptor_extraction(keypoints, object_cloud, descriptors);
}

template<typename DescriptorExtractor, typename DescriptorType>
void Feature_Extractor_3D<DescriptorExtractor, DescriptorType>::get_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                                                                           typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints)
{
    // ** Downsample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (object_cloud);
    vg.setLeafSize (CLOUD_RES, CLOUD_RES, CLOUD_RES);
    vg.filter (*object_downsampled);

    // ** Extract keypoints
    keypoint_extraction(object_downsampled, keypoints);
}


template<typename DescriptorExtractor, typename DescriptorType>
void Feature_Extractor_3D<DescriptorExtractor, DescriptorType>::keypoint_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints)
{
    // ** Subsample cloud since there is not really texture in cloud
    std::clock_t begin = clock();

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (object_cloud);
    vg.setLeafSize (KEYPOINT_RES, KEYPOINT_RES, KEYPOINT_RES);
    vg.filter (*keypoints);

    std::clock_t end = clock();
    ROS_INFO("Keypoints [%.3f ms]. Before: %lu points. After: %lu points",
             RAS_Utils::time_diff_ms(begin, end), object_cloud->size(), keypoints->size());
}

template<typename DescriptorExtractor, typename DescriptorType>
void Feature_Extractor_3D<DescriptorExtractor, DescriptorType>::descriptor_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints,
                                              const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloud,
                                           typename pcl::PointCloud<DescriptorType>::Ptr &descriptors)
{
    std::clock_t begin = clock();

    // ** First, compute normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod
        (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    normal_estimation.setRadiusSearch (3 * KEYPOINT_RES); // Tune this
    normal_estimation.setInputCloud (object_cloud);
    normal_estimation.compute (*cloud_normals);

    double t_normals = RAS_Utils::time_diff_ms(begin,clock());
    begin = clock();

    // ** Second, compute descriptors
    DescriptorExtractor extractor;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    extractor.setSearchMethod (tree);

    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    extractor.setRadiusSearch (5 * KEYPOINT_RES); // Tune this, has to be larger than the previous one

    descriptors.reset(new pcl::PointCloud<DescriptorType>);

    extractor.setInputCloud (keypoints);
    extractor.setSearchSurface(object_cloud);
    extractor.setInputNormals (cloud_normals);
    extractor.compute (*descriptors);
    ROS_INFO("Descriptors: %lu", descriptors->size());
    double t_descriptors = RAS_Utils::time_diff_ms(begin, clock());

    ROS_INFO("Descriptors [Normal: %.3f, Descriptor: %.3f, TOTAL: %.3f ms]", t_normals, t_descriptors, t_normals + t_descriptors);
}


#endif // FEATURE_EXTRACTOR_H
